function obj = retaskSensors_external_CRMI_POD(obj, bandMultiplier)
% retaskSensors_external_CRMI_POD
% Greedy sensor retasking using *band-limited CRMI* computed in POD space,
% using the full reduced covariance Pr (not diagonal-only variances).
%
% - For each candidate patch center:
%   1) Build the set of pixel centers in the patch.
%   2) Keep only pixels within a band around the ego path (polyline).
%   3) Map those pixels to global linear indices lin ⊂ {1..L}.
%   4) Form S = Up(lin,:) (measurement matrix restricted to those pixels).
%   5) Compute MI score: 0.5 * logdet( I + Pr * (S' S)/Rmeas ).
%
% - Then greedily assign patches to sensors that just measured, excluding:
%   - any sensor's current locationIndex
%   - any sensor's current destinationNode
%
% Inputs:
%   bandMultiplier (optional) : band radius factor in pixels (world units = multiplier*res).
%                              default = 2.

    if nargin < 2 || isempty(bandMultiplier)
        bandMultiplier = 2;
    end

    % --- POD handles (required) ---
      Up = obj.POD.Up;   % [L×q]
    Pr = obj.POD.Pr;   % [q×q]
    q  = size(Up,2);

    % --- Precompute constants & geometry ---
    patchSize = 256;
    halfSize  = patchSize/2;

    res     = obj.environment.resolution;           % world units per pixel
    bounds  = obj.environment.domainLimits;         % [xMin xMax; yMin yMax]
    sz      = size(obj.environment.trueState.Temp); % [ny nx]
    centers = obj.sensorGrid.coordinates;           % [2×P]
    P       = size(centers, 2);

    % Band radius (world)
    bandRad = bandMultiplier * res;

    % Relative pixel-center offsets within a patch (world coords)
    offs = ((1:patchSize) - halfSize - 0.5) * res;  % matches your pixel-center convention
    [XO, YO] = meshgrid(offs, offs);                % [ps×ps]
    xyRel = [XO(:)'; YO(:)'];                       % [2×ps^2]

    % Ego path polyline (world coords)
    nodeIds = obj.currentPath(:).';
    if numel(nodeIds) < 1
        error('retaskSensors_external_CRMI_POD:NoPath', 'obj.currentPath is empty.');
    end
    pathXY = obj.egoGrid.coordinates(:, nodeIds);   % [2×K]

    % Measurement noise scalar for CRMI score (kept consistent with your SMI usage)
    Rmeas = single(0.01);

    % --- 1) Compute POD-CRMI score for each patch (band-limited) ---
    CRMI = zeros(1, P, 'single');

    for p = 1:P
        c = centers(:, p);

        % Absolute pixel centers for this patch
        absXY = c + xyRel;                          % [2×ps^2]

        % Distance from each pixel-center to the ego path
        dPix = minDistPointsToPolyline(absXY, pathXY);  % [1×ps^2]

        % Keep only pixels within band
        keep = (dPix <= bandRad);
        if ~any(keep)
            CRMI(p) = 0;
            continue
        end

        % Convert kept world coords to global image indices
        xIdx = round((absXY(1,keep) - bounds(1,1)) / res) + 1;   % cols
        yIdx = round((absXY(2,keep) - bounds(2,1)) / res) + 1;   % rows

        inBounds = (xIdx>=1 & xIdx<=sz(2) & yIdx>=1 & yIdx<=sz(1));
        if ~any(inBounds)
            CRMI(p) = 0;
            continue
        end

        xIdx = xIdx(inBounds);
        yIdx = yIdx(inBounds);

        % Linear indices into full state vector (column-major by MATLAB)
        lin = sub2ind(sz, yIdx, xIdx);             % [M×1]
        lin = lin(:);

        % Form measurement matrix restricted to these pixels
        % S = C_p * Up, implemented as row selection
        S = Up(lin, :);                            % [M×q]
        if isempty(S)
            CRMI(p) = 0;
            continue
        end

        % Compute G = S' S / Rmeas  (q×q)
        % (use single arithmetic for speed/consistency)
        G = (S' * S) / Rmeas;                      % [q×q]

        % MI score: 0.5 * logdet( I + Pr * G )
        A = eye(q, 'single') + single(Pr) * G;     % [q×q]
        A = (A + A') * 0.5;                        % symmetrize

        % logdet via Cholesky with jitter fallback
        [Lchol, pFlag] = chol(A, 'lower');
        if pFlag > 0
            jitter = single(1e-6);
            [Lchol, pFlag2] = chol(A + jitter*eye(q,'single'), 'lower');
            if pFlag2 > 0
                % As last resort, score this patch as 0 (numerical issue)
                CRMI(p) = 0;
                continue
            end
        end
        logdetA = 2 * sum(log(diag(Lchol)));
        CRMI(p) = 0.5 * logdetA;
    end

    % --- 2) Availability mask: disallow current locations and current destinations ---
    locIdx  = arrayfun(@(s) s.locationIndex,   obj.sensors);
    destIdx = arrayfun(@(s) s.destinationNode, obj.sensors);
    disallowed = unique([locIdx, destIdx]);

    available = true(1, P);
    disallowed = disallowed(disallowed>=1 & disallowed<=P);
    available(disallowed) = false;

    % --- 3) Greedy retasking for sensors with new measurements ---
    gamma = 0.5;              % blend weight between distance-to-sensor and distance-to-ego-next
    numS  = numel(obj.sensors);

    % Guard ego-next (if path length 1, use current node)
    if numel(obj.currentPath) >= 2
        egoNext = obj.currentPath(2);
    else
        egoNext = obj.currentPath(1);
    end
    eNextXY = obj.egoGrid.coordinates(:, egoNext);

    for i = 1:numS
        sensor = obj.sensors(i);
        if ~sensor.hasMeasurement
            continue
        end

        % Sensor's current grid-center position (use its current node index)
        sPos = centers(:, sensor.locationIndex);

        % Distances from every candidate center:
        d1 = sqrt(sum((centers - sPos  ).^2, 1));     % to sensor
        d2 = sqrt(sum((centers - eNextXY).^2, 1));    % to ego-next
        d_all = gamma * d1 + (1 - gamma) * d2;

        candIdx = find(available);
        if isempty(candIdx)
            warning('retaskSensors_external_CRMI_POD:NoPatch', ...
                    'Sensor %d: no valid patch available for retasking.', i);
            continue
        end

        d_min = min(d_all(candIdx));
        d_max = max(d_all(candIdx));

        if d_max > d_min
            alpha = max(CRMI(candIdx)) / (d_max - d_min);
        else
            alpha = 0;
        end

        reward = CRMI + alpha * (d_min - d_all);

        [~, k] = max(reward(candIdx));
        best   = candIdx(k);

        obj.sensors(i) = sensor.retask(best);
        available(best) = false;
    end
end

% -------- helper: minimal distance from many points to a polyline ----------
function d = minDistPointsToPolyline(Pts, Poly)
% Pts:  [2×N] points (world coords)
% Poly: [2×K] ordered vertices of the path polyline
% d  :  [1×N] minimal Euclidean distance point-to-polyline

    N = size(Pts,2);
    K = size(Poly,2);

    if K == 0
        d = inf(1,N);
        return
    elseif K == 1
        diff = Pts - Poly;
        d = sqrt(sum(diff.^2,1));
        return
    end

    d = inf(1,N);
    A  = Poly(:,1:end-1);       % [2×(K-1)]
    B  = Poly(:,2:end);         % [2×(K-1)]
    AB = B - A;

    for s = 1:(K-1)
        a  = A(:,s);
        ab = AB(:,s);
        ap = Pts - a;                       % [2×N]
        denom = sum(ab.^2);                 % scalar
        if denom <= 0, continue; end

        t = (ab' * ap) ./ denom;            % [1×N]
        t = max(0, min(1, t));
        proj = a + ab * t;                  % [2×N]
        diff = Pts - proj;
        d = min(d, sqrt(sum(diff.^2,1)));
    end
end
