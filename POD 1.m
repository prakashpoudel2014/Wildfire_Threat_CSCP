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
q = 50;                         % truncation parameter
LX_q = U(:,1:q);                % first q columns = principal components

% --- 6) Compute compressed latent vector x_tilda ---
% For each snapshot x_t:
X_tilda = LX_q' * Xc;           % q x nt
% Now, X_tilda(:,k) = x_tilda at time step k

% --- 7) Reconstruct full state ---
Xr = LX_q * X_tilda + x_mean;   % N x nt
Xr_spatial = reshape(Xr, nx, ny, nt);

% --- 8) Optional: check energy captured ---
lambda_X = diag(S).^2;          % eigenvalues
gamma_x = sum(lambda_X(1:q)) / sum(lambda_X);  % fraction of energy captured
fprintf('Energy captured by first %d modes: %.4f\n', q, gamma_x);

% --- 9) Visualization ---
time_idx = 2000;
figure;
subplot(1,2,1);
imagesc(reshape(X(:,time_idx), nx, ny)); axis image; colorbar; title('Original State');
subplot(1,2,2);
imagesc(Xr_spatial(:,:,time_idx)); axis image; colorbar; title('Reconstructed State');

error = zeros(1,nt);

for i = 1:nt

error(i) = norm(Xr_spatial(:,:,i)-BB.Temp(:,:,i))/norm(BB.Temp(:,:,i));

end

figure
plot(1:nt,error)


