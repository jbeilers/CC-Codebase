% solve_lqr_gains.m
%
% Solves the discrete LQR gain vector for the CoreXY gantry control loop.
%
% The LQRController uses an augmented error state per axis:
%   x_aug = [ error,  error_dot,  integral_error ]
%
% Control law (as implemented in LQRController.cpp):
%   u = Kp*e + Kd*de + Ki*int_e
%
% The plant is a double integrator (point mass):
%   pos[k+1] = pos[k] + vel[k]*dt
%   vel[k+1] = vel[k] + u[k]*dt
%
% Error dynamics (feedforward handles the reference trajectory,
% so the LQR only sees residual error):
%   e[k+1]     = e[k] + de[k]*dt
%   de[k+1]    = de[k] - u[k]*dt
%   int_e[k+1] = int_e[k] + e[k]*dt
%
% This gives the augmented discrete system:
%   F_aug = [1,  dt,  0 ]      B_aug = [  0  ]
%           [0,   1,  0 ]               [ -dt ]
%           [dt,  0,  1 ]               [  0  ]
%
% Both X and Y axes share the same gains (symmetric gantry assumption).
%
% HOW TO USE
% ----------
%   1. Update the parameters in the SYSTEM PARAMETERS section.
%   2. Tune Q and R in the TUNING section to shape the response.
%   3. Run — the script prints the K vector and closed-loop poles.
%   4. Copy the printed K values into Constants.h LQR::K.
%
% TUNING GUIDE
% ------------
%   Q(1,1) — penalises position error.   Increase to tighten tracking.
%   Q(2,2) — penalises velocity error.   Increase to dampen oscillation.
%   Q(3,3) — penalises integral error.   Increase to eliminate steady-state offset.
%             Set to 0 to disable integral action entirely (recommended first).
%   R      — penalises control effort.   Increase if acceleration commands are too large.
%
%   Start with Q(3,3) = 0 and R set so that peak accel ≈ MAX_ACCEL_MMS2.
%   Once P+D response is clean, gradually increase Q(3,3) to add integral.
%   Note: Ki must be POSITIVE for stable integral action in this implementation.

clc; clear;

% ================================================================
%  SYSTEM PARAMETERS — update these to match Constants.h
% ================================================================
dt          = 0.001;        % Control loop period (s)  = 1/CONTROL_LOOP_HZ
mass_kg     = 1.5;          % Gantry moving mass (kg)  = Gantry::MOVING_MASS_KG
max_vel     = 300.0;        % Max velocity (mm/s)       = Trajectory::MAX_VEL_MMS
max_accel   = 500.0;        % Max acceleration (mm/s²)  = Trajectory::MAX_ACCEL_MMS2

% ================================================================
%  TUNING — adjust these to shape the response
% ================================================================
% State cost weights: [position error, velocity error, integral error]
% Units:  1/mm²,  1/(mm/s)²,  1/(mm·s)²
% pos_tol_mm sets the error magnitude at which LQR saturates the accel limit:
%   saturation_threshold = max_accel / Kp  ≈  pos_tol_mm (roughly)
% At 300mm/s with FF handling the nominal path, residual tracking errors
% during a move are typically 5–15mm — so pos_tol should be in that range.
% Tighten only after confirming stable tracking with loose values.
pos_tol_mm   = 8.0;    % Acceptable position error (mm)
vel_tol_mms  = 40.0;   % Acceptable velocity error (mm/s)  — scale with max_vel
int_tol      = 50.0;   % Acceptable integral (mm·s)        — large: Ki disabled for now

Q = diag([ 1/pos_tol_mm^2, ...
           1/vel_tol_mms^2, ...
           1/int_tol^2 ]);

% Control effort weight: 1/(mm/s²)²
% Set so that a typical tracking error drives at most ~max_accel/4
accel_budget = max_accel / 4;   % mm/s²
R = 1 / accel_budget^2;

% ================================================================
%  BUILD DISCRETE AUGMENTED SYSTEM
% ================================================================
% Augmented state: [e, de, int_e]
% e     = ref_pos  - est_pos   (mm)
% de    = ref_vel  - est_vel   (mm/s)
% int_e = integral of e * dt   (mm·s)

F_aug = [ 1,  dt,  0  ; ...
          0,  1,   0  ; ...
          dt, 0,   1  ];

B_aug = [  0  ; ...
          -dt  ; ...
           0  ];

% ================================================================
%  SOLVE DISCRETE LQR
% ================================================================
% dlqr returns K for the law  u = -K*x  (note the minus sign).
% LQRController.cpp uses      u = +K*x  (positive convention).
% Therefore negate K before using in the firmware.
[K_dlqr, ~, ~] = dlqr(F_aug, B_aug, Q, R);

K_code = -K_dlqr;   % gains to copy into Constants.h

Kp = K_code(1);
Kd = K_code(2);
Ki = K_code(3);

% ================================================================
%  VERIFY STABILITY
% ================================================================
% Closed-loop under u = -K_dlqr*x  is  F - B*K_dlqr  (standard form).
F_cl = F_aug - B_aug * K_dlqr;
cl_poles = eig(F_cl);
stable = all(abs(cl_poles) < 1.0);

% ================================================================
%  RESULTS
% ================================================================
fprintf('\n=== LQR SOLUTION ===\n');
fprintf('  Kp = %.6f\n', Kp);
fprintf('  Kd = %.6f\n', Kd);
fprintf('  Ki = %.6f\n', Ki);
fprintf('\n=== CLOSED-LOOP POLES ===\n');
for i = 1:length(cl_poles)
    fprintf('  z%d = %.6f + %.6fj   |z| = %.6f\n', ...
        i, real(cl_poles(i)), imag(cl_poles(i)), abs(cl_poles(i)));
end
fprintf('\nStable: %s\n', string(stable));

if ~stable
    warning('Closed-loop system is UNSTABLE. Increase R or decrease Q.');
end

% ================================================================
%  PRINT Constants.h SNIPPET
% ================================================================
fprintf('\n=== Copy into Constants.h ===\n');
fprintf('constexpr float K[1][3] = {\n');
fprintf('    { %.6ff, %.6ff, %.6ff }\n', Kp, Kd, Ki);
fprintf('};\n');

% ================================================================
%  SIMULATE STEP RESPONSE (1 mm step, no reference trajectory)
% ================================================================
N = 3000;   % 3 seconds at 1 kHz
x = zeros(3, N+1);   % [e, de, int_e]
x(:,1) = [1; 0; 0];  % 1 mm initial position error

u_hist = zeros(1, N);
for k = 1:N
    u_hist(k) = K_code * x(:,k);   % same law as firmware: u = +K_code*x
    % Clamp to accel budget for realism
    u_hist(k) = max(-max_accel, min(max_accel, u_hist(k)));
    x(:,k+1) = F_aug * x(:,k) + B_aug * u_hist(k);
end

t = (0:N) * dt;

figure('Name', 'LQR Step Response (1 mm error)', 'NumberTitle', 'off');
tiledlayout(3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

nexttile;
plot(t, x(1,:), 'r-', 'LineWidth', 1.5);
yline(0, 'k--', 'LineWidth', 0.8);
xlabel('Time (s)');  ylabel('Position error (mm)');
title('Position Error — Step Response');
grid on;

nexttile;
plot(t, x(2,:), 'b-', 'LineWidth', 1.5);
yline(0, 'k--', 'LineWidth', 0.8);
xlabel('Time (s)');  ylabel('Velocity error (mm/s)');
title('Velocity Error');
grid on;

nexttile;
plot(t(1:end-1), u_hist, 'k-', 'LineWidth', 1.5);
yline(0, 'k--', 'LineWidth', 0.8);
yline( max_accel, 'r--', 'LineWidth', 0.8, 'DisplayName', '+MAX\_ACCEL');
yline(-max_accel, 'r--', 'LineWidth', 0.8, 'DisplayName', '-MAX\_ACCEL');
xlabel('Time (s)');  ylabel('Accel command (mm/s²)');
title('Control Effort');
legend('Location', 'best');
grid on;

% Report 2% settling time
tol = 0.02;
settled_idx = find(abs(x(1,:)) < tol, 1, 'first');
if ~isempty(settled_idx)
    fprintf('\n2%% settling time: %.3f s\n', t(settled_idx));
else
    fprintf('\nDid not settle to 2%% within %.1f s\n', t(end));
end
