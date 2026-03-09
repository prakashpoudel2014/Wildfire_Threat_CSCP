function obj = retaskSensors_external(obj)
    % RETASKSENSORSEXTERNAL  Greedy assignment using CRMI + distance,
    % excluding each sensor's own location and any already‐assigned destinations.

    % --- Precompute constants ---
    patchSize = 256;
    halfSize  = patchSize/2;
    res       = obj.environment.resolution;
    bounds    = obj.environment.domainLimits;          % [xMin xMax; yMin yMax]
    sz        = size(obj.environment.trueState.Temp);  % [ny×nx]
    centers   = obj.sensorGrid.coordinates;            % [2×P]
    P         = size(centers, 2);

    % Unpack covariance map
    Var = reshape(obj.estimate.Covariance, sz);        % [ny×nx]

    % Offsets within one patch (flattened)
    offs    = ((1:patchSize) - halfSize - 0.5) * res;
    [XO, YO] = meshgrid(offs, offs);
    xyRel   = [XO(:)'; YO(:)'];                        % [2×(patchSize^2)]

    % Actor's future path coords
    pathCoords = obj.egoGrid.coordinates(:, obj.currentPath);

    % Gaussian kernel width for CRMI
    sigma = (patchSize * res) / 2;

    % --- 1) Compute raw CRMI for each patch center ---
    rawCRMI = zeros(1, P, 'like', Var);
    for p = 1:P
        c = centers(:, p);
        d2        = sum((pathCoords - c).^2, 1);
        relevance = sum(exp(-d2 / (2 * sigma^2)));

        % Find variance of all pixels relevant to the
        absXY = c + xyRel;
        xIdx  = round((absXY(1,:) - bounds(1,1)) / res) + 1;
        yIdx  = round((absXY(2,:) - bounds(2,1)) / res) + 1;
        valid = xIdx>=1 & xIdx<=sz(2) & yIdx>=1 & yIdx<=sz(1);
        if any(valid)            
            rawCRMI(p) = relevance * mean(Var([xIdx,yIdx]));
        end
    end

    % --- 2) Build availability mask ---
    locIdx  = arrayfun(@(s) s.locationIndex,   obj.sensors);
    destIdx = arrayfun(@(s) s.destinationNode, obj.sensors);
    disallowed = unique([locIdx, destIdx]);
    available  = true(1, P);
    available(disallowed) = false;

    % --- 3) Greedy retasking for sensors with a new measurement ---
    gamma      = 0.5;
    numS       = numel(obj.sensors);
    for i = 1:numS
        sensor = obj.sensors(i);
        if ~sensor.hasMeasurement
            continue
        end

        % Combine distances to sensor and to actor
        sPos = centers(:, sensor.locationIndex);
        eNext = obj.egoGrid.coordinates(:, obj.currentPath(2));
        d1 = sqrt(sum((centers - sPos)  .^2, 1));
        d2 = sqrt(sum((centers - eNext) .^2, 1));
        d_all = gamma * d1 + (1-gamma) * d2;

        % Normalize distance term
        d_min = min(d_all(available));
        d_max = max(d_all(available));
        alpha = max(rawCRMI(available)) / (d_max - d_min);

        % Compute reward over available patches
        f_all    = d_min - d_all;
        reward   = rawCRMI + alpha * f_all;

        % Restrict to still-available patches
        candIdx = find(available);
        [~, loc] = max(reward(candIdx));
        best     = candIdx(loc);

        % Assign and mark unavailable
        obj.sensors(i) = sensor.retask(best);
        available(best) = false;
    end
end
