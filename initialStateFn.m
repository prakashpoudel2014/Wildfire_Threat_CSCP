function trueState = initialStateFn(imagePath, domainLimits, resolution, numfires)
    % Derived domain parameters
    dx = resolution;
    x = domainLimits(1,1):dx:domainLimits(1,2);
    y = domainLimits(2,1):dx:domainLimits(2,2);
    mapSize = [length(y), length(x)];

    % N and R used for reference scaling
    N = (domainLimits(1,2) - domainLimits(1,1)) / 2;
    R = resolution;
    p = 0.3;

    % rng(69420); % 69420 used for BB  
    rng(9000); % Reproducible ignition was 9000

    % Generate fuel map from image
    gridSize = round(2*N/R) + 1;
    RGB = imread(imagePath);
    RGB = imresize(RGB, [gridSize, gridSize]);
    gray = double(im2gray(RGB)) / 256;
    baseFuel = 1 - gray;
    baseFuel = baseFuel * (30 - 2) + 2;

    fuel = imresize(baseFuel, mapSize, 'nearest');

    % Allocate GPU arrays
    Temp = gpuArray(zeros(mapSize, 'single'));
    fuel = gpuArray(single(fuel));

    % Random ignition points
    fire_locs = [randi(mapSize(1), [numfires, 1]), randi(mapSize(2), [numfires, 1])];
    for i = 1:numfires
        Temp(fire_locs(i,1), fire_locs(i,2)) = 5;
    end

    % Output true state
    trueState = struct();
    trueState.Temp = Temp;
    trueState.fuel = fuel;
    trueState.fire_locs = fire_locs;
    trueState.p = p;
    trueState.mapSize = mapSize;
end