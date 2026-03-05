classdef (Abstract) SensorNetwork
    % SensorNetwork - Abstract superclass for coordinating sensors and
    % estimation in a dynamic environment with a mobile actor.
    %
    % This class defines the general interface for:
    % - Managing distributed sensors
    % - Updating state estimates with intermittent observations
    % - Recomputing optimal paths for the actor
    % - Retasking sensors based on updated context
    % - Defining traversable grids for sensors and the actor

    properties (Abstract)
        sensors             % Array of Sensor objects
        estimate            % Belief about environment (map, vector, etc.)
        environment         % Handle to Environment object
        currentPath         % Current planned path (current → goal)
        historicPath        % Historical path of actor (start → current)

        egoGrid             % Navigable grid or graph for the ego vehicle
        sensorGrid          % Navigable grid or graph for sensor movement
    end

    methods (Abstract)
        obj = updateEstimate(obj, obsList, posList)
        % Update belief state with (or without) new sensor observations
        % obsList: cell array or matrix of observations
        % posList: world positions corresponding to obsList

        path = computeOptimalPath(obj, goal)
        % Recompute optimal actor path to goal using current stateEstimate

        obj = retaskSensors(obj)
        % Reassign sensor destinations based on updated estimate and path

        obj = buildEgoGrid(obj)
        % Construct the grid over which the actor can plan paths

        obj = buildSensorGrid(obj)
        % Construct the grid or graph over which sensors may move
    end
end
