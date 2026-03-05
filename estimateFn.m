function [xUp, covUp, fuelOut, fireLocsOut] = estimateFn( ...
    estimate, covDiag, rgbPatch, bounds, wind, Q, environment, fuelEst, fireLocsEst)
% ESTIMATEFN  One‐step predict & diagonal‐EKF‐correct for fire temperature,
% with segmentation plotting, measurement‐driven fire/fuel correction,
% and adjusted flame‐confidence and fuel resets.

    % Grid metadata
    sz  = size(environment.trueState.Temp);   % [ny×nx]
    res = environment.resolution;
    dom = environment.domainLimits;           % [xMin xMax; yMin yMax]

    %% 1) SEGMENTATION, PLOTTING & FIRE/FUEL CORRECTION
    if ~isempty(rgbPatch)
        % Load UNet once
        persistent segNet
        if isempty(segNet)
            tmp    = load('unet_final_model.mat','net');
            segNet = tmp.net;
        end

        % Softmax probabilities directly from RGB patch
        probs = predict(segNet, rgbPatch);    % [256×256×4]

        % Hard‐classification map
        [~, cls] = max(probs, [], 3);         % 1=flame,2=scar,3=forest,4=undef

        % % Plot segmentation classes
        % cmap = [ 1.0, 0.5, 0.0;    % flame → orange
        %          0.0, 0.0, 0.0;    % scar  → black
        %          0.0, 1.0, 0.0;    % forest→ green
        %          1.0, 1.0, 1.0 ];  % undef → white
        % figure(42); clf;
        % imagesc(cls);
        % colormap(cmap);
        % clim([1 4]);
        % colorbar(...
        %   'Ticks', 1:4, ...
        %   'TickLabels', {'Flame','Scar','Forest','Undefined'} ...
        % );
        % title('UNet Segmentation Classes');
        % axis image off;
        % drawnow;
        
        % Map patch → global indices (flip Y for top→bottom)
        xvec = bounds(1,1):res:bounds(1,2)-res;
        yvec = bounds(2,2)-res:-res:bounds(2,1);
        [Xg, Yg] = meshgrid(xvec, yvec);
        rIdx = round((Yg - dom(2,1))/res) + 1;
        cIdx = round((Xg - dom(1,1))/res) + 1;

        % Forest (class 3): remove from fireLocs & reset fuel to 20
        forestMask = (cls == 3);
        if any(forestMask(:))
            pts = [rIdx(forestMask), cIdx(forestMask)];
            fireLocsEst = setdiff(fireLocsEst, pts, 'rows');
            idxF = sub2ind(sz, pts(:,1), pts(:,2));
            fuelEst(idxF) = 20;
        end

        % Scar (class 2): remove from fireLocs, set temp later to 1°C
        scarMask = (cls == 2);
        if any(scarMask(:))
            pts = [rIdx(scarMask), cIdx(scarMask)];
            fireLocsEst = setdiff(fireLocsEst, pts, 'rows');
        end

        % Flame (class 1) only high‐confidence (>0.75): add to fireLocs
        flameMask = (cls == 1) & (probs(:,:,1) > 0.75);
        if any(flameMask(:))
            pts = [rIdx(flameMask), cIdx(flameMask)];
            fireLocsEst = unique([fireLocsEst; pts], 'rows');
        end
    end

    %% 2) PREDICT via fireStepFn
    S0.Size      = sz(1);
    S0.Temp      = reshape(estimate, sz);
    S0.fuel      = fuelEst;
    S0.fire_locs = fireLocsEst;
    S0.p         = environment.trueState.p;

    [S1, ~]           = fireStepFn(S0, wind, [], []);
    xPred        = reshape(S1.Temp, [], 1);
    varPred      = covDiag + Q;       % process‐noise diagonal
    fuelOut      = S1.fuel;
    fireLocsOut  = S1.fire_locs;

    %% 3) EKF CORRECTION (if measurement present)
    if ~isempty(rgbPatch)
        % Build observation map:
        %   flame → 5×prob, scar → 1°C, forest → 0°C
        obsMap = zeros(size(cls), 'single');
        % flame
        hcFlame = flameMask;  % reuse mask from above
        probs1 = probs(:,:,1);
        obsMap(hcFlame) = 5 * probs1(hcFlame);
        % scar
        obsMap(scarMask) = 0;
        % (forest and undef remain 0)

        obs    = obsMap(:);

        % Per‐pixel measurement variance
        classVals = single([5, 1, 0, 0]);
        sqVals    = reshape(classVals.^2, 1,1,[]);
        E2        = sum(probs .* sqVals, 3);  
        E1        = sum(probs .* reshape(classVals,1,1,[]), 3);
        varMap    = E2 - E1.^2;
        Rvec      = varMap(:);

        % Indices of measured pixels
        idxVec    = sub2ind(sz, rIdx(:), cIdx(:));

        % Innovation & gain
        zPred = xPred(idxVec);
        innov = obs   - zPred;
        % K     = varPred(idxVec) ./ (varPred(idxVec) + Rvec);
        K     = varPred(idxVec) ./ (varPred(idxVec) + mean(Rvec) + eps);

        % Apply update
        xUp      = xPred;
        xUp(idxVec)    = xPred(idxVec) + K .* innov;
        covUp    = varPred;
        covUp(idxVec) = (1 - K) .* varPred(idxVec);

    else
        % No measurement → pure prediction
        xUp   = xPred;
        covUp = varPred;
    end
end
