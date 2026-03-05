classdef ForestFireEnvironment < Environment
    properties
        windField         % [T x 2] matrix of wind vectors (vx, vy)
        windIndex         % current time index into windField
        stateHistory      % used to save full environment state

        imageGenFn        % CGAN image generator: RGB = f(IR, label)
        initialStateFn    % no-arg initializer for trueState
        fireStepFn        % fire dynamics update function

        baseMap             % Optical reference image, used to simulate IR
        fireIntensityGain   % Multiplier for converting Temp to IR brightness
    end

    methods
        function obj = ForestFireEnvironment(domainLimits, resolution, imagePath, numfires, windField)
            % --- Core environment parameters ---
            obj.domainLimits = domainLimits;
            obj.resolution = resolution;
            obj.time = 0;
            obj.baseMap = imread(imagePath);
            obj.fireIntensityGain = 20;  % Or configurable

            % --- Grid setup ---
            dx = resolution;
            x = domainLimits(1,1):dx:domainLimits(1,2);
            y = domainLimits(2,1):dx:domainLimits(2,2);
            obj.mapSize = [length(y), length(x)];

            obj.stateHistory.Temp = [];% zeros([obj.mapSize, 3000], 'single');
            obj.stateHistory.fuel = [];% zeros([obj.mapSize, 3000], 'single');
            obj.stateHistory.fire_locs = [];% cell(3000, 1);

            % --- Wind ---
            obj.windField = windField;
            obj.windIndex = 1;

            % --- Initialization function (as closure) ---
            obj.initialStateFn = @() initialStateFn(imagePath, domainLimits, resolution, numfires);
            obj.trueState = obj.initialStateFn();

            % --- Fire dynamics function (defined internally) ---
            obj.fireStepFn = @(trueState, wind, stateHistory, k) fireStepFn(trueState, wind, stateHistory, k);

            % --- CGAN image generator ---
            obj.imageGenFn = @imageGenFn;
        end

        function obj = step(obj, dt)
            % Advance simulation using current wind
            wind = obj.windField(obj.windIndex, :);
            timestep = obj.time/dt+1;
            [obj.trueState, obj.stateHistory] = obj.fireStepFn(obj.trueState, wind, obj.stateHistory, timestep);
            obj.time = obj.time + dt;

            % Advance wind step (hold at last value if done)
            if obj.windIndex < size(obj.windField, 1)
                obj.windIndex = obj.windIndex + 1;
            end
        end

        function obs = getObservation(obj, pos)
            [obs, ~] = obj.sampleImagePatch(pos);
        end

        function [rgbPatch, bounds] = sampleImagePatch(obj, pos)
            % SAMPLEIMAGEPATCH – pull a RGB patch at world‐coord pos
            [irPatch, label, bounds] = obj.extractIRPatch(pos);
            rgbPatch = obj.imageGenFn(irPatch, label);
            % figure
            % imshow(rgbPatch)
            % drawnow
            % figure
            % imshow(irPatch)
            % drawnow
        end
    
        function [irPatch, label, bounds] = extractIRPatch(obj, pos)
            % EXTRACTIRPATCH  Extracts a 256×256 IR patch around pos
            %   • Temperature patch is sliced around true world‐Y and then
            %     flipped vertically so its values align with the real map.
            %   • Background patch remains centered on flipped world‐Y but
            %     is never flipped in orientation.
            %   • Returns the CGAN‐ready irPatch, its binary label, and world bounds.
    
            patchSize = 256;
            halfSize  = patchSize/2;              % =128
            res       = obj.resolution;
            dom       = obj.domainLimits;         % [xMin xMax; yMin yMax]
    
            % 1) Compute X→column once
            colpix = round((pos(1) - dom(1,1)) / res) + 1;
    
            % 2) Compute two different Y→row centers
            rowpix_temp = round((pos(2) - dom(2,1)) / res) + 1;  % true temp center
            rowpix_bg   = round((dom(2,2) - pos(2)) / res) + 1;  % background center
    
            [nRows, nCols] = size(obj.trueState.Temp);
    
            % 3) Define the slicing windows
            r0_t = rowpix_temp - halfSize + 1;  r1_t = rowpix_temp + halfSize;
            r0_b = rowpix_bg   - halfSize + 1;  r1_b = rowpix_bg   + halfSize;
            c0   = colpix       - halfSize + 1;  c1   = colpix       + halfSize;
    
            % 4) Check bounds
            if r0_t < 1 || r0_b < 1 || r1_t > nRows || r1_b > nRows || ...
               c0   < 1 || c1   > nCols
                error('extractIRPatch:OutOfBounds', ...
                      'Pos (%.3f,%.3f) too close to edge for a %dx%d patch.', ...
                      pos(1), pos(2), patchSize, patchSize);
            end
    
            % 5) Extract the temperature patch and flip it vertically
            tempPatch = gather( ...
                obj.trueState.Temp(r0_t:r1_t, c0:c1) );
            tempPatch = flipud(tempPatch);  % flip only the temp values
    
            % 6) Extract the background patch (never flipped)
            baseMapGray = rgb2gray( ...
                imresize(obj.baseMap, [nRows, nCols]) );
            baseGray    = 255 - baseMapGray;
            basePatch   = single( ...
                baseGray(r0_b:r1_b, c0:c1) );
    
            % 7) Build the IR image + shimmer noise + normalize
            IR_raw   = basePatch + obj.fireIntensityGain * tempPatch;
            noise    = randn(size(tempPatch)) .* (tempPatch > 0);
            noise    = noise .* (tempPatch ./ max(tempPatch(:)+eps)) * 25;
            IR_noisy = IR_raw + noise;
            mn       = min(IR_noisy(:)); 
            mx       = max(IR_noisy(:));
            IR_norm  = (IR_noisy - mn) / (mx - mn + eps);
            irPatch  = uint8(IR_norm * 255);
    
            % 8) Binary fire label (>0.1% hot pixels)
            label = single(nnz(tempPatch > 0) / numel(tempPatch) > 0.001);
    
            % 9) World‐coordinate bounds (shared for both layers)
            halfWorld = halfSize * res;
            bounds    = [ pos(1)-halfWorld, pos(1)+halfWorld;
                          pos(2)-halfWorld, pos(2)+halfWorld ];
        end

        function coords = getCoordinates(obj)
            % Return [2 x N] list of coordinates in world space
            [X, Y] = meshgrid(...
                linspace(obj.domainLimits(1,1), obj.domainLimits(1,2), obj.mapSize(2)), ...
                linspace(obj.domainLimits(2,1), obj.domainLimits(2,2), obj.mapSize(1)));
            coords = [X(:)'; Y(:)'];
        end

        function obj = reset(obj)
            % Re-initialize environment state
            obj.trueState = obj.initialStateFn();
            obj.time = 0;
            obj.windIndex = 1;
        end

        function [rowIdx, colIdx] = worldToPatchIndices(obj, pos)
            % WORLD TOPATCHINDICES
            %   pos = [x; y] in world coords
            %   returns two vectors rowIdx, colIdx each of length 256
            patchSize = 256;
            % half‐size of patch in pixels
            halfPix   = patchSize/2;      % =128
            % half‐size in world units
            halfWorld = halfPix * obj.resolution;
    
            % compute the world‐coordinate bounds of the patch
            xMinW = pos(1) - halfWorld;  
            xMaxW = pos(1) + halfWorld;
            yMinW = pos(2) - halfWorld;  
            yMaxW = pos(2) + halfWorld;
    
            % convert to floating‐point pixel indices
            % col = (x - xMin)/dx + 1
            cMin = (xMinW - obj.domainLimits(1,1)) / obj.resolution + 1;
            rMin = (yMinW - obj.domainLimits(2,1)) / obj.resolution + 1;
            % now round to nearest integer pixel
            c0   = round(cMin);
            r0   = round(rMin);
    
            % make sure we never go out of the array bounds
            if r0 < 1 || (r0 + 2*halfPix-1) > size(obj.trueState.Temp,1) || ...
               c0 < 1 || (c0 + 2*halfPix-1) > size(obj.trueState.Temp,2)
                error('sampleImagePatch:OutOfBounds', ...
                      'Position (%.3f,%.3f) too close to edge to extract a full %dx%d patch.', ...
                      pos(1), pos(2), 2*halfPix, 2*halfPix);
            end
    
            % Build the index vectors
            rowIdx = r0 : (r0 + 2*halfPix-1);
            colIdx = c0 : (c0 + 2*halfPix-1);
        end
    end
end
