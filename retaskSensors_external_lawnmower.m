function obj = retaskSensors_external_lawnmower(obj)
    % --- Build sweep paterns assuming square grid---
    persistent sweep1 sweep2 fwd1 fwd2

    if isempty(sweep1)
       centers      = obj.sensorGrid.coordinates;            
       P            = size(centers, 2);
       points       = reshape(1:P,[sqrt(P),sqrt(P)]);
       evenRows     = 2:2:size(points,1); 
       points(evenRows,:) = fliplr(points(evenRows,:));
       sweep1 = reshape(points',[1,P]);
    end

    if isempty(sweep2)
       points       = reshape(1:P,[sqrt(P),sqrt(P)]);
       points       = points';
       evenRows     = 2:2:size(points,1); 
       points(evenRows,:) = fliplr(points(evenRows,:));
       sweep2 = reshape(points',[1,P]);
    end

    if isempty(fwd1)
        fwd1 = true;
    end

    if isempty(fwd2)
        fwd2 = true;
    end

    % --- Build availability mask ---
    locIdx      = arrayfun(@(s) s.locationIndex,   obj.sensors);

    % --- Random retasking for sensors with a new measurement ---
    numS       = numel(obj.sensors);
    for i = 1:numS
        sensor = obj.sensors(i);
        if ~sensor.hasMeasurement
            continue
        end
       

        if i == 1
            if fwd1
                isEnd = locIdx(1)==sweep1(end);
                if isEnd
                    fwd1 = false;
                    newLoc = sweep1(end-1);
                else
                    newLoc = sweep1(find(sweep1 == locIdx(1)) + 1);
                end
            else
                isEnd = locIdx(1)==sweep1(1);
                if isEnd
                    fwd1 = true;
                    newLoc = sweep1(2);
                else
                    newLoc = sweep1(find(sweep1 == locIdx(1)) - 1);
                end
            end
        else
             if fwd2
                isEnd = locIdx(2)==sweep2(end);
                if isEnd
                    fwd2 = false;
                    newLoc = sweep2(end-1);
                else
                    newLoc = sweep2(find(sweep2 == locIdx(2)) + 1);
                end
            else
                isEnd = locIdx(2)==sweep2(1);
                if isEnd
                    fwd2 = true;
                    newLoc = sweep2(2);
                else
                    newLoc = sweep2(find(sweep2 == locIdx(2)) - 1);
                end
            end

        end

        % Assign newloc
        obj.sensors(i) = sensor.retask(newLoc);

    end
end
