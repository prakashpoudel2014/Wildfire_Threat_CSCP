function obj = retaskSensors_external_CRMI(obj, bandMultiplier)
% Greedy retasking using CRMI computed *pixel-wise*:
% For each patch, only pixels whose centers lie within (bandMultiplier * R)
% of the ego path contribute to the information score.
%
% Inputs
%   bandMultiplier  (optional) width factor for the path band, default = 2.
%                   Effective radius = bandMultiplier * environment.resolution.

    if nargin < 2
        bandMultiplier = 2;     % "2R" band by default
    end

    % --- Precompute constants & geometry ---
    patchSize = 256;
    halfSize  = patchSize/2;
    Rworld    = obj.environment.resolution;           % grid spacing (world units / pixel)
    bandRad   = bandMultiplier * Rworld;              % distance threshold around path
    bounds    = obj.environment.domainLimits;         % [xMin xMax; yMin yMax]
    sz        = size(obj.environment.trueState.Temp); % [ny x nx]
    centers   = obj.sensorGrid.coordinates;           % [2 x P] patch centers (world coords)
    P         = size(centers, 2);

    % Variance field (vector state laid out as ny×nx image)
    Var = reshape(obj.estimate.Covariance, sz);       % [ny x nx] (double)

    % Relative pixel-center offsets within a patch (world coords)
    offs = ((1:patchSize) - halfSize - 0.5) * Rworld;
    [XO, YO] = meshgrid(offs, offs);                  % each is [ps x ps]
    xyRel = [XO(:)'; YO(:)'];                         % [2 x ps^2]

    % Ego path polyline (world coords) from currentPath indices
    nodeIds = obj.currentPath(:).';
    pathXY  = obj.egoGrid.coordinates(:, nodeIds);    % [2 x K]

    % --- 1) Pixel-wise CRMI (SMI over in-band pixels only) per patch ---
    CRMI = zeros(1, P);
    Rmeas = .01;                                        % measurement noise scalar (as in your SMI)
    for p = 1:P
        c = centers(:, p);

        % Absolute pixel centers for this patch
        absXY = c + xyRel;                            % [2 x ps^2]

        % Distance from each pixel-center to the ego path
        dPix  = minDistPointsToPolyline(absXY, pathXY);   % [1 x ps^2]

        % Keep only pixels within the band
        keep  = (dPix <= bandRad);
        if ~any(keep)
            CRMI(p) = 0;      % no path-adjacent pixels in this patch
            continue
        end

        % Convert world coords to image indices for kept pixels
        xIdx = round((absXY(1,keep) - bounds(1,1)) / Rworld) + 1;  % columns
        yIdx = round((absXY(2,keep) - bounds(2,1)) / Rworld) + 1;  % rows

        inBounds = xIdx>=1 & xIdx<=sz(2) & yIdx>=1 & yIdx<=sz(1);
        if ~any(inBounds)
            CRMI(p) = 0;
            continue
        end

        xIdx = xIdx(inBounds);
        yIdx = yIdx(inBounds);

        lin  = sub2ind(sz, yIdx, xIdx);
        vPix = Var(lin);                               % variances for in-band pixels

        % SMI on the *subset* of pixels (numerically guarded)
        % Pzz computed from in-band pixels only (your "pixel-wise reduction")
        Pzz   = sum(vPix) + Rmeas;
        numer = max(realmin, vPix);
        denom = max(realmin, vPix - (vPix.^2)./Pzz);
        CRMI(p) = 0.5 * sum(log(numer) - log(denom));
    end

    % --- 2) Build availability mask ---
    locIdx  = arrayfun(@(s) s.locationIndex,   obj.sensors);
    destIdx = arrayfun(@(s) s.destinationNode, obj.sensors);
    disallowed = unique([locIdx, destIdx]);
    available  = true(1, P);
    available(disallowed) = false;
    
    % --- 3) Greedy retasking for sensors that just measured ---
    gamma = 0.5;                          % distance blend weight
    numS  = numel(obj.sensors);

    for i = 1:numS
        sensor = obj.sensors(i);
        if ~sensor.hasMeasurement
            continue
        end

        % Blend of distance to this sensor and to ego's next node
        sPos  = centers(:, sensor.locationIndex);
        eNext = obj.egoGrid.coordinates(:, obj.currentPath(2));

        d1 = sqrt(sum((centers - sPos ).^2, 1));      % to sensor
        d2 = sqrt(sum((centers - eNext).^2, 1));      % to ego-next
        d_all = gamma * d1 + (1 - gamma) * d2;

        % Normalize distance over currently available patches
        candIdx = find(available);
        if isempty(candIdx), break; end

        d_min = min(d_all(candIdx));
        d_max = max(d_all(candIdx));
        if d_max > d_min
            alpha = max(CRMI(candIdx)) / (d_max - d_min);
        else
            alpha = 0;
        end

        % Reward: pixel-wise CRMI minus normalized distance penalty
        reward = CRMI + alpha * (d_min - d_all);

        % Choose best available
        [~, k] = max(reward(candIdx));
        best   = candIdx(k);

        obj.sensors(i) = sensor.retask(best);
        available(best) = false;          % prevent reuse this round
    end
end

% -------- helper: minimal distance from many points to a polyline ----------
function d = minDistPointsToPolyline(Pts, Poly)
% Pts:  [2 x N] points (world coords)
% Poly: [2 x K] ordered vertices of the path polyline
% d  :  [1 x N] minimal Euclidean distance point-to-polyline

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
    A = Poly(:,1:end-1);       % segment starts  [2 x (K-1)]
    B = Poly(:,2:end);         % segment ends    [2 x (K-1)]
    AB = B - A;                % segment vectors

    for s = 1:(K-1)
        a  = A(:,s);
        ab = AB(:,s);
        ap = Pts - a;                          % [2 x N]
        denom = sum(ab.^2);                    % scalar
        if denom <= 0, continue; end
        t = (ab' * ap) ./ denom;               % [1 x N], projection param
        t = max(0, min(1, t));                 % clamp to segment
        proj = a + ab * t;                     % closest points on segment
        diff = Pts - proj;                     % [2 x N]
        d    = min(d, sqrt(sum(diff.^2,1)));
    end
end