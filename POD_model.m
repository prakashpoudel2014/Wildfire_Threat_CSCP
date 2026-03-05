
% % 1. Reshape data matrix into a 2D matrix (flatten spatial grid)
% [nx, ny, nt] = size(AA.Temp);
% X_2d = reshape(AA.Temp, nx*ny, nt);
% 
% % 2. Mean center the data (subtract the mean at each time step)
% X_2d_centered = X_2d - mean(X_2d, 2);  % Subtract column-wise mean
% 
% % 3. Perform Singular Value Decomposition (SVD) to get POD modes
% [U, S, V] = svd(X_2d_centered, 'econ');
% 
% % 4. Select the number of POD modes for the reduced-order model
% % Choose a number of modes based on the energy retention criterion
% num_modes = 100;  % Adjust this depending on how much you want to reduce
% 
% % 5. Construct the reduced-order model using the selected POD modes
% U_red = U(:, 1:num_modes);  % Select the first 'num_modes' POD modes
% S_red = S(1:num_modes, 1:num_modes);  % Corresponding singular values
% V_red = V(:, 1:num_modes);  % Time evolution coefficients
% 
% % 6. Reduced data matrix (approximation of the original data)
% X_red_approx = U_red * S_red * V_red';  % Reconstruct the reduced data
% 
% % 7. Reshape the reduced data back into a spatial grid (for visualization or further use)
% X_red_approx_spatial = reshape(X_red_approx, nx, ny, nt);
% 
% % Plotting the first few POD modes (optional)
% figure;
% for i = 1:5  % Show the first 5 POD modes
%     subplot(1, 5, i);
%     imagesc(reshape(U_red(:, i), nx, ny));  % Reshape the POD mode back to spatial grid
%     colorbar;
%     title(['POD Mode ' num2str(i)]);
% end
% 
% % Plot the energy capture (singular values)
% figure;
% plot(diag(S), 'o-', 'LineWidth', 2);
% xlabel('Mode Index');
% ylabel('Singular Value');
% title('Singular Values (Energy Capture)');
% 
% % Calculate the cumulative energy capture
% cumulative_energy = cumsum(diag(S).^2) / sum(diag(S).^2);
% figure;
% plot(cumulative_energy, 'LineWidth', 2);
% xlabel('Number of Modes');
% ylabel('Cumulative Energy');
% title('Cumulative Energy Capture with POD Modes');
% 
% % Saving the reduced-order model (optional)
% save('POD_model.mat', 'U_red', 'S_red', 'V_red');

% --- NEW CODE FOR IMAGE COMPARISON ---

% % Choose a time step for comparison (e.g., time step 1000)
% time_step = 1000;
% 
% % Extract the true data at time_step 1000
% X_true_at_t1000 = reshape(AA.Temp(:, :, time_step), nx, ny);  % True data at time step 1000
% 
% % Extract the reduced-order model at time_step 1000
% X_red_at_t1000 = reshape(X_red_approx(:, time_step), nx, ny);  % Reduced-order model at time step 1000
% 
% % Plot the comparison between true data and reduced-order model at time step 1000
% figure;
% subplot(1, 2, 1);
% imagesc(X_true_at_t1000);  % Plot the true data
% colorbar;
% title(['True Data at Time Step ' num2str(time_step)]);
% 
% subplot(1, 2, 2);
% imagesc(X_red_at_t1000);  % Plot the reduced-order model data
% colorbar;
% title(['Reduced-Order Model at Time Step ' num2str(time_step)]);
% 
% % Add a title for the comparison plot
% sgtitle(['Comparison of True Data and Reduced-Order Model at Time Step ' num2str(time_step)]);



%% -------------------------
% POD following the theory: X_tilda, LX_q, reconstruction
% Input: BB.Temp (nx x ny x nt)
%% -------------------------
 close all; clc;

[nx, ny, nt] = size(BB.Temp);
N = nx*ny;

% --- Flatten data: each column is a snapshot x_t ---
X = reshape(BB.Temp, N, nt);  % N x nt

% --- 1) Compute mean ---
x_mean = mean(X, 2);           % mean over snapshots

% --- 2) Center data ---
Xc = X - x_mean;                % X_c = X - mean

% --- 3) Covariance matrix ---
% Empirical covariance (N x N) -- may be large
%Cx = (1/(nt-1)) * (Xc * Xc');  % only if N is small
% Instead, we use SVD to avoid huge covariance

% --- 4) SVD of centered data ---
[U, S, V] = svd(Xc,'econ');     % U: N x nt, V: nt x nt, S: nt x nt

% --- 5) Select number of POD modes ---
q = 100;                         % truncation parameter
% LX_q = U(:,1:q);                % first q columns = principal components

% --- 6) Compute compressed latent vector x_tilda ---
% For each snapshot x_t:
% X_tilda = LX_q' * Xc;           % q x nt
X_tilda = LX_q_100' * Xc; 
% Now, X_tilda(:,k) = x_tilda at time step k

% --- 7) Reconstruct full state ---
% Xr = LX_q * X_tilda + x_mean;   % N x nt
Xr = LX_q_100 * X_tilda + x_mean; 
Xr_spatial = reshape(Xr, nx, ny, nt);

% --- 8) Optional: check energy captured ---
lambda_X = diag(S).^2;          % eigenvalues
gamma_x = sum(lambda_X(1:q)) / sum(lambda_X);  % fraction of energy captured
fprintf('Energy captured by first %d modes: %.4f\n', q, gamma_x);

% % Saving the reduced-order model (optional)
% save('POD_model.mat', 'LX_q');

% % --- 9) Visualization ---
% time_idx = 2000;
% figure;
% subplot(1,2,1);
% imagesc(reshape(X(:,time_idx), nx, ny)); axis image; colorbar; title('Original State');
% subplot(1,2,2);
% imagesc(Xr_spatial(:,:,time_idx)); axis image; colorbar; title('Reconstructed State');
% 
% error = zeros(1,nt);
% 
% for i = 1:nt
% 
% error(i) = norm(Xr_spatial(:,:,i)-AA.Temp(:,:,i))/norm(AA.Temp(:,:,i));
% 
% end
% 
% figure
% plot(1:nt,error)

% --- 9) Visualization and Error Analysis ---

% Select time index
time_idx = 2000;

x1 = linspace(-1, 1, nx);
x2 = linspace(-1, 1, ny);

% Original Temperature Field
figure('Color','w');
imagesc(x1, x2, reshape(X(:,time_idx), nx, ny));
axis equal tight;
set(gca,'YDir','normal');
colormap(jet); colorbar;

title('Original Temperature Field', ...
      'Interpreter','latex','FontSize',14);
xlabel('$x_1$','Interpreter','latex','FontSize',18);
ylabel('$x_2$','Interpreter','latex','FontSize',18);
h = colorbar;
ylabel(h,'Temperature $(\mathcal{T})$','Interpreter','latex','FontSize',18);
xlim([-1 1]); ylim([-1 1]);


% POD Reconstructed State
figure('Color','w');
imagesc(x1, x2, flip(Xr_spatial(:,:,time_idx)));
axis image tight; colormap(jet); colorbar;

title('Reconstructed Temperature Field', ...
      'Interpreter','latex','FontSize',14);
xlabel('$x_1$','Interpreter','latex','FontSize',18);
ylabel('$x_2$','Interpreter','latex','FontSize',18);
h = colorbar;
ylabel(h,'Temperature $(\mathcal{T}_{\mathrm{rec}})$','Interpreter','latex','FontSize',18);
xlim([-1 1]); ylim([-1 1]);
max_Xr_spatial = max(max(max(Xr_spatial(:,:,:) )));
normalized_max_Xr_spatial =  Xr_spatial/max_Xr_spatial;
max_BB_temp = max(max(max(BB.Temp(:,:,:) )));
normalized_max_BB_temp =  BB.Temp/max_BB_temp;
% Reconstruction Error over Time
% error = zeros(1, nt);
% for i = 1:nt
%     error(i) = norm(Xr_spatial(:,:,i) - BB.Temp(:,:,i), 'fro') ...
%                / norm(BB.Temp(:,:,i), 'fro');
% end
error = zeros(1, nt);
for i = 1:nt
    error(i) = norm(normalized_max_Xr_spatial(:,:,i) - normalized_max_BB_temp(:,:,i), 'fro') ...
               / norm(normalized_max_Xr_spatial(:,:,i), 'fro');
end
figure('Color','w');
plot(1:nt, error, 'LineWidth', 2);
xlabel('Time index', 'Interpreter','latex','FontSize',16);
ylabel('Reconstruction Error', 'Interpreter','latex','FontSize',16);
title('Relative $L_2$ reconstruction error over time','Interpreter','latex','FontSize',14);
