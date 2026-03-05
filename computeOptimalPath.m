function path = computeOptimalPath(temperatureEstimate, grid, startIdx, goalIdx, weight)
% computeOptimalPath - Dijkstra on an undirected, temperature‐aware grid,
%                     using a while‐loop until the goal is reached.
%
% Throws an error if the goal is unreachable.

    if nargin < 5 || isempty(weight)
        weight = 1000;
    end

    coords   = grid.coordinates;   % [2×N]
    adj      = grid.adjacency;     % [N×N sparse]
    timeCost = grid.cost;          % [N×N sparse]
    N        = size(coords, 2);

    % Reshape to square temp map
    sz   = sqrt(numel(temperatureEstimate));
    % Safeguard for negative values
    temperatureEstimate(temperatureEstimate<0)=0;
    % Reshape
    Temp = reshape(temperatureEstimate, [sz, sz]);

    % Build symmetric threatCost matrix
    threatCost = sparse(N, N);
    tol = 1e-6;
    [iList, jList] = find(adj);
    for k = 1:numel(iList)
        i = iList(k); j = jList(k);
        p1 = coords(:,i); p2 = coords(:,j);
        Ldist = norm(p2-p1);
        nSamp = max(5, ceil(Ldist * sz));  % ~ one sample per pixel
        xs = linspace(p1(1), p2(1), nSamp);
        ys = linspace(p1(2), p2(2), nSamp);

        % Map samples into Temp indices
        xPix = round((xs - min(coords(1,:))) / (max(coords(1,:))-min(coords(1,:))) * (sz-1)) + 1;
        yPix = round((ys - min(coords(2,:))) / (max(coords(2,:))-min(coords(2,:))) * (sz-1)) + 1;
        xPix = min(max(xPix,1),sz);
        yPix = min(max(yPix,1),sz);

        avgThreat = mean(Temp(sub2ind([sz,sz], yPix, xPix)));
        totalCost = timeCost(i,j) + weight * avgThreat;

        % assign both directions
        threatCost(i,j) = totalCost;
        threatCost(j,i) = totalCost;
    end

    % Dijkstra initialization
    dist    = inf(N,1);
    prev    = nan(N,1);
    visited = false(N,1);
    dist(startIdx) = 0;
    tieOccurred = false;
    maxEdge = max(timeCost(:)) + weight * max(Temp(:));

    % Main loop: continue until goal is popped or no reachable nodes remain
    while true
        % pick unvisited node with minimal dist
        candidates = find(~visited);
        [dmin, idxMin] = min(dist(candidates));
        if isinf(dmin)
            error('computeOptimalPath:DisconnectedGraph', ...
                  'No path from %d to %d—graph disconnected.', startIdx, goalIdx);
        end
        u = candidates(idxMin);
        visited(u) = true;

        % if we've reached the goal, stop
        if u == goalIdx
            break;
        end

        % relax all neighbors
        nbrs = find(threatCost(u,:));
        for v = nbrs
            alt = dist(u) + threatCost(u,v);
            if alt < dist(v) - tol
                dist(v) = alt;
                prev(v) = u;
            elseif abs(alt - dist(v)) <= tol
                % tie-break
                if rand() < 0.5
                    prev(v) = u;
                end
                tieOccurred = true;
            end
        end
    end

    % Reconstruct path backwards
    path = goalIdx;
    while path(1) ~= startIdx
        p = prev(path(1));
        path = [p, path];
    end

    % if tieOccurred
    %     warning('computeOptimalPath:TieBreak', ...
    %             'Encountered equal-cost alternatives; random tie-break applied.');
    % end
end
