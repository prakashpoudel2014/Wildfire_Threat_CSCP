%{
SOFTWARE LICENSE
----------------
Copyright (c) 2023 by 
	Raghvendra V Cowlagi
	Jefferey DesRoches
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
Class definition of actor platform (ego vehicle).
%}

classdef Actor < Platform
    properties
        actualPath             % List of completed nodes
        expectedPath           % Remaining path (from current to goal)
        justReached            % logical, set true only on the step you arrive
        edgeDistance           % Length of the current edge
        totalExposure          % Accumulated exposure from true threat
        
        environment            % Handle to the environment
    end

    methods
        function obj = Actor(speed, environment, gridCoords)
            obj.Speed = speed;
            obj.environment = environment;
            obj.coordinateSource = gridCoords;  % [2×N] grid node positions

            % initialize motion state
            obj.Position        = [];
            obj.Heading         = [];
            obj.Velocity        = [];
            obj.travelDistance  = 0;

            % no path yet
            obj.actualPath      = [];
            obj.expectedPath    = [];
            obj.currentEdge     = [];
            obj.edgeDistance    = 0;
            obj.justReached     = false;
            obj.totalExposure   = 0;
        end

        function obj = threatExposure(obj)
            % Integrate true exposure over last movement segment

            if size(obj.Position, 2) < 2
                return  % no movement yet
            end

            prior = obj.Position(:, end-1);
            current = obj.Position(:, end);

            xs = linspace(prior(1), current(1), 5);
            ys = linspace(prior(2), current(2), 5);
            locs = [xs; ys];

            Temp = gather(obj.environment.trueState.Temp);
            bounds = obj.environment.domainLimits;
            res = obj.environment.resolution;

            % Convert to pixel indices
            xpix = round((locs(1,:) - bounds(1,1)) / res) + 1;
            ypix = round((locs(2,:) - bounds(2,1)) / res) + 1;

            sz = size(Temp);
            mask = xpix >= 1 & xpix <= sz(2) & ypix >= 1 & ypix <= sz(1);
            idx = sub2ind(sz, ypix(mask), xpix(mask));
            threatVals = Temp(idx);

            % Integrate exposure across arc
            tvals = linspace(0, 1, numel(threatVals));
            poly = polyfit(tvals, threatVals, 4);
            integral = polyint(poly);

            arcLen = norm(current - prior);
            exposure = (polyval(integral, 1) - polyval(integral, 0)) / arcLen;

            obj.totalExposure = obj.totalExposure + exposure;
        end

        function obj = checkReachedVertex(obj)
            % CHECKREACHEDVERTEX – flag arrival, snap to node, and record it
            obj.justReached = false;
            if obj.travelDistance >= obj.edgeDistance
                % 1) Mark arrival
                obj.justReached    = true;
                obj.travelDistance = 0;
        
                % 2) Snap position to exact waypoint
                coords = obj.coordinateSource;
                nextNode = obj.currentEdge(2);
                obj.Position(:,end) = coords(:, nextNode);
        
                % 3) Record that node in the history
                obj.actualPath = [obj.actualPath, nextNode];
            end
        end

        function obj = updatePath(obj, newPath)
            % UPDATEPATH – Replace the ego's planned route and reset motion
            if numel(newPath) < 2
                return
            end
        
            % 1) Update the future path and current edge
            obj.expectedPath = newPath(2:end);
            obj.currentEdge  = [newPath(1), newPath(2)];
        
            % 2) Recompute edge distance and position
            coords = obj.coordinateSource;
            obj.Position = coords(:, obj.currentEdge(1));
            obj.edgeDistance = norm(coords(:, newPath(2)) - coords(:, newPath(1)));
        
            % 3) Reset heading & velocity to point along the new edge
            h = obj.findHeading();                % new heading
            obj.Heading  = h;
            obj.Velocity = [cos(h); sin(h)] * obj.Speed;
        end

    end
end

