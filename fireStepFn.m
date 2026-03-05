function [trueState, stateHistory] = fireStepFn(trueState, wind, stateHistory, k)
    % Advance fire simulation by one step (dt unused for now)

    % Extract
    Temp = gather(trueState.Temp);
    fuel = gather(trueState.fuel);
    fire_locs = trueState.fire_locs;
    p = trueState.p;

    Size = size(Temp);
    Vel = wind;  % [vx vy]
    fire_new = [];

    % === 1. Spread from hot cells ===
    for i = 1:size(fire_locs,1)
        x = fire_locs(i,1);
        y = fire_locs(i,2);

        if any([x == 1, x == Size(1), y == 1, y == Size(2)])
            continue
        end

        T = Temp(x, y);

        neighbors = [
            x+1, y, Vel(2) < 0;
            x-1, y, Vel(2) > 0;
            x, y+1, Vel(1) > 0;
            x, y-1, Vel(1) < 0
        ];

        for j = 1:4
            xi = neighbors(j,1);
            yi = neighbors(j,2);
            advective = neighbors(j,3);
        
            T2 = Temp(xi, yi);
            
            if j <= 2  % Up/down → y-direction → use Vel(2)
                velComponent = Vel(2);
            else        % Left/right → x-direction → use Vel(1)
                velComponent = Vel(1);
            end
        
            if advective
                Temp(xi, yi) = round(T2 + p * (T - T2) * abs(velComponent), 3);
            else
                Temp(xi, yi) = round(T2 + p^2 * (T - T2), 3);
            end
        end
    end

    % === 2. Identify hot cells ===
    [xh, yh] = find(Temp > 0);
    hot = [xh, yh];

    % === 3. Ignition from temp and fuel ===
    for i = 1:size(hot,1)
        x = hot(i,1);
        y = hot(i,2);
        if rand() < p * Temp(x, y) / 10 && fuel(x, y) > 0
            fire_new = [fire_new; x, y];
        end
    end

    % === 4. Remove duplicates & add new fire ===
    fire_new = unique(fire_new, 'rows');
    if ~isempty(fire_new)
        [inOld, ~] = ismember(fire_new, fire_locs, 'rows');
        fire_new(inOld, :) = [];
    end
    fire_locs = [fire_locs; fire_new];

    % === 5. Burn temperature & fuel ===
    removeIdx = [];
    for i = 1:size(fire_locs, 1)
        x = fire_locs(i,1);
        y = fire_locs(i,2);

        T = Temp(x, y);
        F = fuel(x, y);

        Temp(x, y) = T + p * (10 - T) * F / 100;
        T2 = Temp(x, y);

        fuel(x, y) = F - p * T2;
        F = fuel(x, y);

        if F < 0
            fuel(x, y) = 0;
            Temp(x, y) = T2 - sqrt(p / 10000) * T2;
            if round(Temp(x, y), 2) == 0
                removeIdx = [removeIdx; i];
                Temp(x, y) = 0;
            end
        end
    end

    fire_locs(removeIdx, :) = [];

    % === 6. Return updated state ===
    trueState.Temp = gpuArray(Temp);
    trueState.fuel = gpuArray(fuel);
    trueState.fire_locs = fire_locs;

    % if ~isempty(stateHistory)
    %     stateHistory.Temp(:, :, k) = gather(trueState.Temp);  % single
    %     stateHistory.fuel(:, :, k) = gather(trueState.fuel);  % single
    %     stateHistory.fire_locs{k} = trueState.fire_locs;      % variable size
    % end
end