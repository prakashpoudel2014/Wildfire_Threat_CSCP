function obj = retaskSensors_external_SMI(obj)
    % RETASKSENSORSEXTERNAL  Greedy assignment using SMI + distance,
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

    % --- 1) Compute raw SMI for each patch center ---
    SMI = zeros(1, P);
    for p = 1:P
        c = centers(:, p);

        % Find variance of all pixels relevant to each patch
        absXY = c + xyRel;
        xIdx  = round((absXY(1,:) - bounds(1,1)) / res) + 1;
        yIdx  = round((absXY(2,:) - bounds(2,1)) / res) + 1;
        valid = xIdx>=1 & xIdx<=sz(2) & yIdx>=1 & yIdx<=sz(1);
        if any(valid)            
            Pxx = log(Var(:));
            Pxz = zeros(length(Var(:)),1);
            idx = sub2ind(size(Var),xIdx,yIdx);
            Pxz(idx) = Var(idx);
            Pxz2 = Pxz.^2;
            R = 1;
            Pzz = sum(Pxz)+R;

            denom = log(Var(:)-Pxz2/Pzz);

            SMI(p) = 0.5*(sum(Pxx)-sum(denom));
        end
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
            alpha = max(SMI(candIdx)) / (d_max - d_min);
        else
            alpha = 0;
        end

        % Reward: pixel-wise CRMI minus normalized distance penalty
        reward = SMI + alpha * (d_min - d_all);

        % Choose best available
        [~, k] = max(reward(candIdx));
        best   = candIdx(k);

        obj.sensors(i) = sensor.retask(best);
        available(best) = false;          % prevent reuse this round
    end
end
