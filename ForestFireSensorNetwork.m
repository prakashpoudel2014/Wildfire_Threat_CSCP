classdef ForestFireSensorNetwork < SensorNetwork
    % ForestFireSensorNetwork - Fire-specific SensorNetwork implementation
    % Stores Temp/Fuel/FireLocs with diagonal covariance, builds grids,
    % and supports POD-reduced estimation and SMI-POD retasking.

    properties
        % Abstract-class properties
        sensors                 % [1×S] array of Sensor objects
        estimate                % struct with fields:
                                %   .Temp        [ny×nx] current mean Temp map
                                %   .Fuel        [ny×nx] current fuel map
                                %   .FireLocs    [K×2]   flaming cells (row,col)
                                %   .Covariance  [ny×nx] diag variances of Temp
        environment             % Environment object

        currentPath             % [1×P] node indices (current → goal)
        historicPath            % [1×H] nodes the actor has visited

        egoGrid                 % struct: .coordinates, .adjacency, .cost, .startIdx, .goalIdx
        sensorGrid              % struct: .coordinates

        % Additional properties
        egoSpeed                % scalar speed of the actor
        egoGridResolution       % scalar N for N×N ego grid
        sensorOverlapPercent    % in [0,1) overlap fraction for sensor grid
        Q                       % [L×1] process-noise variances (diagonal)
        R                       % (kept for compatibility; unused in POD flow)
        estimateFn              % handle to POD estimator (estimateFn_POD)

        coverageCount           % [ny×nx] per-pixel measurement count

        % POD container
        POD                     % struct with fields:
                                %   .Up     [L×q] POD basis
                                %   .x_mean [L×1] POD mean state
                                %   .y      [q×1] reduced state
                                %   .Pr     [q×q] reduced covariance
    end

    methods
        function obj = retaskSensors(obj)
            % Delegate to POD-aware SMI retasking
            obj = retaskSensors_external_lawnmower(obj);
        end

        function obj = ForestFireSensorNetwork( ...
                sensors, environment, Q, R, egoSpeed, N, overlapPercent, ...
                estimateFn, start, goal, Up, x_mean, Pr0)
            % FORESTFIRESENSORNETWORK Constructor
            %   sensors         – initial Sensor array (possibly empty)
            %   environment     – ForestFireEnvironment handle
            %   Q               – process-noise diagonal (L×1)
            %   R               – measurement-noise (kept for compat)
            %   egoSpeed        – ego speed
            %   N               – ego-grid resolution (default 11)
            %   overlapPercent  – sensor grid overlap (default 0.5)
            %   estimateFn_POD  – handle to POD estimator
            %   start, goal     – ego start/goal node indices
            %   Up              – [L×q] POD basis
            %   x_mean          – [L×1] POD mean (column vector)
            %   Pr0             – [q×q] initial reduced covariance (optional)

            if nargin < 6,  N = 11;                 end
            if nargin < 7,  overlapPercent = 0.5;   end

            % Core handles & params
            obj.sensors              = sensors;
            obj.environment          = environment;
            obj.Q                    = single(Q(:));       % ensure column
            obj.R                    = R;
            obj.egoSpeed             = egoSpeed;
            obj.egoGridResolution    = N;
            obj.sensorOverlapPercent = overlapPercent;
            obj.estimateFn           = estimateFn;

            % Grid size
            sz = size(environment.trueState.Temp);  % [ny, nx]
            L  = prod(sz);

            % EKF state (full) & diag covariance
            obj.estimate.Temp        = zeros(sz,'single');
            obj.estimate.Fuel        = 20 * ones(sz,'single');
            obj.estimate.FireLocs    = zeros(0,2,'uint32');
            obj.estimate.Covariance  = reshape(obj.Q, sz);   % store as [ny×nx]

            % POD init
            obj.POD.Up     = Up;                        % [L×q]
            obj.POD.x_mean = single(x_mean(:));         % [L×1]
            q              = size(Up,2);

            % Initialize reduced state y to match current full estimate (zeros)
            x0 = zeros(L,1,'single');
            obj.POD.y  = Up' * (x0 - obj.POD.x_mean);   % [q×1]

            % Reduced covariance
            if nargin >= 13 && ~isempty(Pr0)
                obj.POD.Pr = Pr0;
            else
                obj.POD.Pr = eye(q,'single') * 1e-3;
            end

            % Paths
            obj.currentPath  = [];
            obj.historicPath = [];

            % Grids
            obj.egoGrid    = struct();
            obj.sensorGrid = struct();

            % Coverage
            obj.coverageCount = zeros(sz,'single');

            % Build grids and seed path
            obj = obj.buildEgoGrid(start, goal);
            obj = obj.buildSensorGrid();
        end

        function obj = updateEstimate(obj, obsList, wind)
            % UPDATEESTIMATE
            %   obsList = {rgbPatch, bounds} (either may be [] if no meas)
            %   wind    = [wx, wy]

            % 1) Unpack measurement
            rgbPatch = obsList{1};
            bounds   = obsList{2};

            % Accumulate coverage if a patch arrived
            if ~isempty(rgbPatch) && ~isempty(bounds)
                obj = obj.addCoverageFromBounds(bounds);
            end

            % 2) Pull prior full state & POD state
            sz        = size(obj.estimate.Temp);
            x_full    = reshape(obj.estimate.Temp, [], 1);        % [L×1]
            covDiag   = reshape(obj.estimate.Covariance, [], 1);  % [L×1] diag covariance
            fuelMap   = obj.estimate.Fuel;
            fireLocs  = obj.estimate.FireLocs;
            
            % 3) POD estimator call
            [xUp, covUp, fuelUp, fireLocsUp] = obj.estimateFn( ... %,POD
                x_full,              ... % full prior Temp vector [L×1]
                covDiag,             ... % full diag covariance [L×1]
                rgbPatch, bounds,    ... % measurement (or [])
                wind,                ... % wind
                obj.Q,               ... % process-noise diag [L×1]
                obj.environment,     ... % env handle
                fuelMap, fireLocs); %,   ... % prior fuel and fire locations
                % obj);                    % provides obj.POD (Up, x_mean)
            
            % 4) Store updated belief
            obj.estimate.Temp        = reshape(xUp, sz);
            obj.estimate.Covariance  = reshape(covUp, sz);
            obj.estimate.Fuel        = fuelUp;
            obj.estimate.FireLocs    = fireLocsUp;

            % Store reduced outputs
            % obj.POD = POD_;

            % 5) Replan ego path (start = current node → goal)
            if ~isempty(obj.currentPath)
                start = obj.currentPath(1);
                goal  = obj.egoGrid.goalIdx;

                newPath = computeOptimalPath( ...
                    obj.estimate.Temp(:), ... % vectorized Temp
                    obj.egoGrid,          ... % grid
                    start,                ... % start
                    goal,                 ... % goal
                    [] );                      % default weight

                obj.historicPath = [obj.historicPath, start];
                obj.currentPath  = newPath;
            end
        end

        function path = computeOptimalPath(obj, varargin)
            % COMPUTE_OPTIMAL_PATH  Replan ego path using latest estimate.
            switch numel(varargin)
                case 1
                    start = obj.currentPath(1);
                    goal  = varargin{1};
                case 2
                    start = varargin{1};
                    goal  = varargin{2};
                otherwise
                    error('computeOptimalPath:BadArgs', ...
                        'Call as computeOptimalPath(goal) or computeOptimalPath(start,goal).');
            end

            path = computeOptimalPath( ...
                obj.estimate.Temp(:), ... % temperatureEstimate
                obj.egoGrid,          ... % grid
                start,                 ... % startIdx
                goal,                  ... % goalIdx
                [] );                       % weight (default)
        end

        function obj = buildEgoGrid(obj, start, goal)
            % BUILD EGO GRID: N×N undirected grid, column-major numbering.
            N      = obj.egoGridResolution;
            bounds = obj.environment.domainLimits;
            x      = linspace(bounds(1,1), bounds(1,2), N);
            y      = linspace(bounds(2,1), bounds(2,2), N);
            [X, Y] = meshgrid(x, y);

            coords   = [X(:)'; Y(:)'];         % 2×(N^2), column-major
            numNodes = N^2;
            A = sparse(numNodes, numNodes);
            C = sparse(numNodes, numNodes);

            idxMat = reshape(1:numNodes, N, N);

            for row = 1:N
                for col = 1:N
                    idx = idxMat(row, col);

                    if row > 1
                        j = idxMat(row-1, col);
                        A(idx,j) = 1;  A(j,idx) = 1;
                        d = norm(coords(:,idx) - coords(:,j));
                        t = d / obj.egoSpeed;
                        C(idx,j) = t;  C(j,idx) = t;
                    end
                    if row < N
                        j = idxMat(row+1, col);
                        A(idx,j) = 1;  A(j,idx) = 1;
                        d = norm(coords(:,idx) - coords(:,j));
                        t = d / obj.egoSpeed;
                        C(idx,j) = t;  C(j,idx) = t;
                    end
                    if col > 1
                        j = idxMat(row, col-1);
                        A(idx,j) = 1;  A(j,idx) = 1;
                        d = norm(coords(:,idx) - coords(:,j));
                        t = d / obj.egoSpeed;
                        C(idx,j) = t;  C(j,idx) = t;
                    end
                    if col < N
                        j = idxMat(row, col+1);
                        A(idx,j) = 1;  A(j,idx) = 1;
                        d = norm(coords(:,idx) - coords(:,j));
                        t = d / obj.egoSpeed;
                        C(idx,j) = t;  C(j,idx) = t;
                    end
                end
            end

            obj.egoGrid.coordinates = coords;
            obj.egoGrid.adjacency   = A;
            obj.egoGrid.cost        = C;
            obj.egoGrid.startIdx    = start;
            obj.egoGrid.goalIdx     = goal;
            obj.currentPath         = [start, goal];
        end

        function obj = buildSensorGrid(obj)
            % BUILD SENSOR GRID – centers with ≥ sensorOverlapPercent; report actual overlap.
            bounds    = obj.environment.domainLimits;   % [xMin xMax; yMin yMax]
            res       = obj.environment.resolution;
            patchPx   = 256;
            patchW    = patchPx * res;
            overlap   = obj.sensorOverlapPercent;

            halfSz = (patchPx/2) * res;
            Lx_eff = (bounds(1,2)-bounds(1,1)) - 2*halfSz;
            Ly_eff = (bounds(2,2)-bounds(2,1)) - 2*halfSz;

            desiredStep = patchW * (1 - overlap);

            Nx = max(1, ceil(Lx_eff/desiredStep));
            Ny = max(1, ceil(Ly_eff/desiredStep));

            stepX = Lx_eff / Nx;
            stepY = Lx_eff * (Ly_eff/Lx_eff) / Ny;  % equals Ly_eff/Ny

            actualOverlapX = 1 - (stepX / patchW);
            actualOverlapY = 1 - (stepY / patchW);

            fprintf('Requested overlap: %.1f%%  →  Actual overlap X: %.1f%%, Y: %.1f%%\n', ...
                overlap*100, actualOverlapX*100, actualOverlapY*100);

            xs = bounds(1,1) + halfSz : stepX : bounds(1,1) + halfSz + Nx*stepX;
            ys = bounds(2,1) + halfSz : stepY : bounds(2,1) + halfSz + Ny*stepY;
            xs(end) = bounds(1,2) - halfSz;
            ys(end) = bounds(2,2) - halfSz;

            [Xg, Yg] = meshgrid(xs, ys);
            Xrm = reshape(Xg.',1,[]);
            Yrm = reshape(Yg.',1,[]);
            coords = [Xrm; Yrm];

            obj.sensorGrid.coordinates = coords;
        end

        function obj = addCoverageFromBounds(obj, bounds)
            if isempty(bounds); return; end

            sz = size(obj.environment.trueState.Temp);
            if isempty(obj.coverageCount)
                obj.coverageCount = zeros(sz, 'uint16');
            end

            res     = obj.environment.resolution;
            dom     = obj.environment.domainLimits;
            xMinDom = dom(1,1);  yMinDom = dom(2,1);

            xMinP = min(bounds(1,:));  xMaxP = max(bounds(1,:));
            yMinP = min(bounds(2,:));  yMaxP = max(bounds(2,:));

            nx = sz(2);  ny = sz(1);
            ix1 = ceil( (xMinP - xMinDom)/res + 0.5 );
            ix2 = floor((xMaxP - xMinDom)/res + 0.5 );
            iy1 = ceil( (yMinP - yMinDom)/res + 0.5 );
            iy2 = floor((yMaxP - yMinDom)/res + 0.5 );

            ix1 = max(1, min(nx, ix1));  ix2 = max(1, min(nx, ix2));
            iy1 = max(1, min(ny, iy1));  iy2 = max(1, min(ny, iy2));

            if ix2 < ix1 || iy2 < iy1
                return
            end

            mask = false(sz);
            mask(iy1:iy2, ix1:ix2) = true;

            obj.coverageCount = obj.coverageCount + mask;
        end

        function plotCoverage(obj)
            if isempty(obj.coverageCount)
                warning('coverageCount is empty. No measurements recorded.');
                return;
            end

            Cmat = obj.coverageCount;
            sz   = size(Cmat);
            xrg  = obj.environment.domainLimits(1,:);
            yrg  = obj.environment.domainLimits(2,:);
            xv   = linspace(xrg(1), xrg(2), sz(2));
            yv   = linspace(yrg(1), yrg(2), sz(1));

            cmax = max(Cmat(:));
            nlev = max(1, ceil(cmax))+1;
            levels = linspace(0, cmax, nlev);

            f=figure; clf;
            contourf(xv, yv, Cmat, levels, 'LineStyle','none');
            axis image; axis xy; colormap("copper");
            xticks([-1 -0.5 0 0.5 1])
            yticks([-1 -0.5 0 0.5 1])
            cbar = colorbar('eastoutside');
            cbar.Label.FontWeight = 'bold';
            fontsize(f,24,"points");
            fontname(f,"Times New Roman");
        end
    end
end

