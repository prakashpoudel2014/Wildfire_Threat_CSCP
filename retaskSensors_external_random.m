function obj = retaskSensors_external_random(obj)
    % --- Build availability mask ---
    locIdx      = arrayfun(@(s) s.locationIndex,   obj.sensors);
    destIdx     = arrayfun(@(s) s.destinationNode, obj.sensors);
    disallowed  = unique([locIdx, destIdx]);
    centers     = obj.sensorGrid.coordinates;            % [2×P]
    P           = size(centers, 2);
    available   = true(1, P);
    available(disallowed) = false;

    % --- Random retasking for sensors with a new measurement ---
    numS       = numel(obj.sensors);
    for i = 1:numS
        sensor = obj.sensors(i);
        if ~sensor.hasMeasurement
            continue
        end

        % Restrict to still-available patches
        candIdx = find(available);
        
        newIdx = randi(length(candIdx));
        newLoc = candIdx(newIdx);

        % Assign and mark unavailable
        obj.sensors(i) = sensor.retask(newLoc);
        available(newLoc) = false;
    end
end
