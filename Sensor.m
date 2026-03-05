%{
SOFTWARE LICENSE
----------------
Copyright (c) 2023 by 
	Raghvendra V Cowlagi
	Bejamin Cooper
	Prakash Poudel

Permission is hereby granted to any person obtaining a copy of this
software and associated documentation files (the "Software"), to deal in
the Software, including the rights to use, copy, modify, merge, copies of
the Software, and to permit persons to whom the Software is furnished to do
so, subject to the following conditions:  

* The above copyright notice and this permission notice shall be included
in all copies or substantial portions of the Software.
* The Software, and its copies or modifications, may not be distributed,
published, or sold for profit. 
* The Software, and any substantial portion thereof, may not be copied or
modified for commercial or for-profit use.

The software is provided "as is", without warranty of any kind, express or
implied, including but not limited to the warranties of merchantability,
fitness for a particular purpose and noninfringement. In no event shall the
authors or copyright holders be liable for any claim, damages or other
liability, whether in an action of contract, tort or otherwise, arising
from, out of or in connection with the software or the use or other
dealings in the software.      


PROGRAM DESCRIPTION
-------------------
Class definition of sensor platform.
%}

classdef Sensor < Platform
    % Sensor - Mobile sensing agent that moves from A to B and collects observations

    properties
        sensorNetwork       % Handle to SensorNetwork (for grid and coordination)
        environment         % Handle to Environment (for observation sampling)

        destinationNode     % Index of current destination node
        locationIndex       % Index of current node
        edgeDistance        % Distance of current edge

        hasMeasurement      % Flag indicating if sensor is ready to return a measurement
        image               % Last captured RGB image (optional)
    end

    methods
        function obj = Sensor(speed, startIdx, destinationIdx, sensorNetwork, environment)
            obj.Speed = speed;
            obj.sensorNetwork = sensorNetwork;
            obj.environment = environment;

            obj.coordinateSource = sensorNetwork.sensorGrid.coordinates;
            obj.Position = obj.coordinateSource(:, startIdx);
            obj.Velocity = [];
            obj.Heading = [];
            obj.travelDistance = 0;
            obj.image = [];

            obj.locationIndex = startIdx;
            obj.hasMeasurement = false;

            if ~isempty(destinationIdx)
                obj.destinationNode = destinationIdx;
                obj.currentEdge = [startIdx, destinationIdx];

                obj.edgeDistance = norm(...
                    obj.coordinateSource(:, destinationIdx) - ...
                    obj.coordinateSource(:, startIdx));

                % Initialize heading and velocity
                obj.Heading = obj.findHeading();
                obj.Velocity = [cos(obj.Heading); sin(obj.Heading)] * obj.Speed;
            else
                obj.destinationNode = [];
                obj.currentEdge = [];
                obj.edgeDistance = 0;
            end
        end

        function obj = checkReachedDestination(obj)
            if obj.travelDistance >= obj.edgeDistance
                obj.travelDistance = 0;
                obj.locationIndex = obj.destinationNode;
                obj.hasMeasurement = true;
        
                % --- SNAP to the exact destination node ---
                coords = obj.coordinateSource;
                obj.Position(:,end) = coords(:, obj.destinationNode);
            end
        end

        function [obj, rgbPatch, bounds] = sampleObservation(obj)
            if ~obj.hasMeasurement
                rgbPatch = []; bounds = [];
                return
            end
            pos = obj.Position(:,end);
            [rgbPatch, bounds] = obj.environment.sampleImagePatch(pos);
            obj.image         = rgbPatch;
            
        end

        function obj = retask(obj, newDestination)
            % Assign a new destination (from SensorNetwork)
            obj.destinationNode = newDestination;
            obj.currentEdge     = [obj.locationIndex, newDestination];
        
            obj.edgeDistance    = norm(...
                obj.coordinateSource(:,newDestination) - ...
                obj.coordinateSource(:,obj.locationIndex));
        
            obj.Heading         = obj.findHeading();
            obj.Velocity        = [cos(obj.Heading); sin(obj.Heading)] * obj.Speed;
        
            % Prevent an immediate re‐sample — only sample again once you re‐arrive
            obj.hasMeasurement  = false;
        end

    end
end

