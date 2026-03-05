function RGB = imageGenFn(irPatch, label)
    persistent dlnetG

    if isempty(dlnetG)
        data = load('checkpoint_epoch_090.mat', 'dlnetG');
        dlnetG = data.dlnetG;
    end

    % --- Check input size ---
    if isempty(irPatch) || any(size(irPatch) ~= [256, 256])
        error('imageGenFn:InvalidInput', 'irPatch must be 256x256 and nonempty.');
    end

    % Convert IR patch to single and normalize
    ir = single(irPatch) / 255;
    
    % Create second channel with same type
    labelChannel = label * ones(size(ir), 'like', ir);
    
    % Concatenate along 3rd dim
    input = cat(3, ir, labelChannel);  % size 256x256x2
    
    % Wrap in dlarray
    dlInput = dlarray(input, 'SSC');

    % --- Predict ---
    dlRGB = predict(dlnetG, dlInput);

    if isempty(dlRGB)
        error('imageGenFn:EmptyOutput', 'CGAN output is empty.');
    end

    % --- Format output ---
    RGB = extractdata(dlRGB);              % 256x256x3
    % Scale and clip to uint8 range
    RGB = uint8(255 * min(max(RGB, 0), 1));
end