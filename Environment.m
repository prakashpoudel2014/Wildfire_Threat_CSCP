classdef (Abstract) Environment
    properties
        domainLimits     % [D × 2] bounds
        resolution       % scalar or vector, optional
        mapSize          % if applicable
        time             % simulation time
        trueState        % raw dynamic state
    end

    methods (Abstract)
        step(obj, dt)                      % Evolve environment
        obs = getObservation(obj, pos)     % Return observable data at pos
        coords = getCoordinates(obj)       % Waypoints or node positions
    end
end