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
Class definition of platform (for actor and sensor motion).
%}

classdef Platform
    properties
        Position            % [2 x T] time-stamped position history
        Speed               % Scalar movement speed
        Velocity            % [2 x T] velocity vector history
        Heading             % [1 x T] angle over time
        travelDistance      % Cumulative distance along current edge
        currentEdge         % [i1, i2] indices of nodes being traversed
        coordinateSource    % [2 x N] node positions (e.g., egoGrid or sensorGrid)
    end

    methods
        function obj = movePlatform(obj, time_step)
            % Integrate motion along currentEdge for one time step

            obj.Heading  = [obj.Heading, obj.findHeading()];
            headingVec   = [cos(obj.Heading(end)); sin(obj.Heading(end))];

            obj.Velocity = [obj.Velocity, headingVec * obj.Speed];
            newPos       = obj.Position(:,end) + obj.Velocity(:,end) * time_step;

            obj.Position = [obj.Position, newPos];
            obj.travelDistance = obj.travelDistance + obj.Speed * time_step;
        end

        function heading = findHeading(obj)
            % Compute angle from currentEdge using coordinateSource

            p1 = obj.coordinateSource(:, obj.currentEdge(1));
            p2 = obj.coordinateSource(:, obj.currentEdge(2));

            heading = atan2(p2(2) - p1(2), p2(1) - p1(1));
        end
    end
end