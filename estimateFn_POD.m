function [xUp, covUp, fuelOut, fireLocsOut, POD] = estimateFn_POD( ...
    estimate, covDiag, rgbPatch, bounds, wind, Q, environment, fuelEst, fireLocsEst, obj)
% ESTIMATEFN_POD
% Matches estimateFn for state propagation + diagonal measurement update.
% Additionally updates POD.Pr (and POD.y) using pretrained POD model in obj.POD.
% POD does NOT affect the estimate update.

    % Grid metadata
    sz  = size(environment.trueState.Temp);
    res = environment.resolution;
    dom = environment.domainLimits;
    L   = prod(sz);

    % Ensure column vectors
    estimate = estimate(:);
    covDiag  = covDiag(:);
    Q        = single(Q(:));

    % POD model (assumed present/correct)
    Up     = obj.POD.Up;         % [L x q]
    x_mean = obj.POD.x_mean(:);  % [L x 1]
    q      = size(Up,2);

    %% 1) SEGMENTATION + FIRE/FUEL CORRECTION (same as estimateFn)
    probs = [];
    cls   = [];
    flameMask = [];
    scarMask  = [];
    rIdx = [];
    cIdx = [];

    if ~isempty(rgbPatch)
        persistent segNet
        if isempty(segNet)
            tmp    = load('unet_final_model.mat','net');
            segNet = tmp.net;
        end

        probs = predict(segNet, rgbPatch);      % [256×256×4]
        [~, cls] = max(probs, [], 3);           % 1=flame,2=scar,3=forest,4=undef

        % Map patch → global indices (flip Y)
        xvec = bounds(1,1):res:bounds(1,2)-res;
        yvec = bounds(2,2)-res:-res:bounds(2,1);
        [Xg, Yg] = meshgrid(xvec, yvec);
        rIdx = round((Yg - dom(2,1))/res) + 1;
        cIdx = round((Xg - dom(1,1))/res) + 1;

        % Forest: remove fire locs and reset fuel
        forestMask = (cls == 3);
        if any(forestMask(:))
            pts = [rIdx(forestMask), cIdx(forestMask)];
            fireLocsEst = setdiff(fireLocsEst, pts, 'rows');
            idxF = sub2ind(sz, pts(:,1), pts(:,2));
            fuelEst(idxF) = 20;
        end

        % Scar: remove fire locs
        scarMask = (cls == 2);
        if any(scarMask(:))
            pts = [rIdx(scarMask), cIdx(scarMask)];
            fireLocsEst = setdiff(fireLocsEst, pts, 'rows');
        end

        % Flame: high confidence only
        flameMask = (cls == 1) & (probs(:,:,1) > 0.75);
        if any(flameMask(:))
            pts = [rIdx(flameMask), cIdx(flameMask)];
            fireLocsEst = unique([fireLocsEst; pts], 'rows');
        end
    end

    %% 2) PREDICT via fireStepFn (same as estimateFn)
    S0.Size      = sz(1);
    S0.Temp      = reshape(estimate, sz);
    S0.fuel      = fuelEst;
    S0.fire_locs = fireLocsEst;
    S0.p         = environment.trueState.p;

    [S1, ~]      = fireStepFn(S0, wind, [], []);
    xPred        = reshape(S1.Temp, [], 1);
    fuelOut      = S1.fuel;
    fireLocsOut  = S1.fire_locs;

    varPred = covDiag + Q;   % diagonal process noise (same as estimateFn)

    % POD predicted reduced covariance: PrPred = Up' * diag(varPred) * Up
    PrPred = Up' * (Up .* varPred);
    PrPred = 0.5 * (PrPred + PrPred');

    %% 3) MEASUREMENT UPDATE (same x/cov behavior as estimateFn) + POD.Pr update
    if ~isempty(rgbPatch)
        % Observation map
        obsMap = zeros(size(cls), 'single');
        probs1 = probs(:,:,1);

        obsMap(flameMask) = 5 * probs1(flameMask);
        obsMap(scarMask)  = 0;
        obs = obsMap(:);

        % Measurement variance
        classVals = single([5, 1, 0, 0]);
        sqVals    = reshape(classVals.^2, 1,1,[]);
        E2        = sum(probs .* sqVals, 3);
        E1        = sum(probs .* reshape(classVals,1,1,[]), 3);
        varMap    = E2 - E1.^2;
        Rvec      = varMap(:);

        % Measured indices
        idxVec = sub2ind(sz, rIdx(:), cIdx(:));

        % Diagonal gain update (mean(Rvec) like estimateFn)
        zPred = xPred(idxVec);
        innov = obs - zPred;

        Rbar = mean(Rvec) + eps('single');
        K    = varPred(idxVec) ./ (varPred(idxVec) + Rbar);

        xUp         = xPred;
        xUp(idxVec) = xPred(idxVec) + K .* innov;

        covUp         = varPred;
        covUp(idxVec) = (1 - K) .* varPred(idxVec);

        % POD posterior covariance update (R ≈ Rbar*I)
        Hr = Up(idxVec, :);
        Jr = (1 / double(Rbar)) * (Hr' * Hr);
        Pr = inv(inv(PrPred) + Jr);
        Pr = 0.5 * (Pr + Pr');

    else
        xUp   = xPred;
        covUp = varPred;
        Pr    = PrPred;
    end

    % Reduced coordinates (for CRMI/SMI; no feedback to estimate)
    y = Up' * (single(xUp) - single(x_mean));

    % Package POD outputs
    POD.Up     = Up;
    POD.x_mean = single(x_mean);
    POD.y      = single(y);
    POD.Pr     = single(Pr);
end
