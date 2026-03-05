% --- Config ---
domainLimits = [-1 1; -1 1];
resolution = 0.002;
numfires = 5;
nsteps = 1000;
imagePath = 'Map1.png';
fireIntensityGain = 20;

% --- Wind setup ---
theta = [linspace(-3*pi/4, pi/4, nsteps/2), linspace(pi/4, -3*pi/4, nsteps/2)]';
windField = [cos(theta), sin(theta)];

% --- Initialize Environment ---
env = ForestFireEnvironment(domainLimits, resolution, imagePath, numfires, windField);
env.fireIntensityGain = fireIntensityGain;

% --- Get valid X range for scanning ---
patchRadius = 256 * resolution / 2;
xMin = domainLimits(1,1) + patchRadius;
xMax = domainLimits(1,2) - patchRadius;

scanXs = linspace(xMin, xMax, nsteps / 25);
fixedY = domainLimits(2,1) + patchRadius;

% --- Create output folder ---
outDir = 'test_RGB_outputs';
if ~exist(outDir, 'dir'); mkdir(outDir); end

% --- Main loop ---
stepIdx = 1;
scanIdx = 1;

for k = 1:nsteps 
    env = env.step(0.002);  % Step environment (dt not used)

    if mod(k, 25) == 0
        pos = [scanXs(scanIdx), fixedY];

        try
            [RGB, label] = env.sampleImagePatch(pos);

            % Save result
            outName = sprintf('%s/frame_%03d_label%d.jpg', outDir, scanIdx, label);
            imwrite(RGB, outName);

            fprintf('Step %d → Image %d saved\n', k, scanIdx); 
            scanIdx = scanIdx + 1;
        catch ME
            warning("Skipping frame %d due to error: %s", scanIdx, ME.message);
        end
    end
end 
