% plot_control_sim.m
%
% Plots CSV output from the Teensy control stack simulation test.
%
% HOW TO USE
% ----------
%   1. Flash teensy41_test, open the serial monitor, send 'G'.
%   2. Wait for "# Done." to appear in the monitor.
%   3. Copy the full serial output (including the header line and all
%      CSV rows) and paste it into a plain text file.
%   4. Set DATA_FILE below to the path of that text file.
%   5. Run this script (F5 or "Run" in the MATLAB editor).
%
% CSV COLUMNS (as printed by test_control_sim.cpp)
% -------------------------------------------------
%   t_ms     — elapsed time since move start (ms)
%   ref_x/y  — trajectory planner reference position (mm)
%   sim_x/y  — simulated plant position (mm)
%   est_x/y  — Kalman filter position estimate (mm)
%   accel_x/y — total acceleration command to plant (mm/s²)
%   err_x/y  — tracking error: ref - sim (mm)

clc; clear; close all

DATA_FILE  = 'D:\School\School Materials\Senior Design\New Codebase\CC Codebase\src\testing\sim_data.txt';
FIG_FOLDER = 'D:\School\School Materials\Senior Design\New Codebase\CC Codebase\figures';   % folder to save PNGs

% ----------------------------------------------------------------
%  Load & parse
% ----------------------------------------------------------------
fid = fopen(DATA_FILE, 'r');
if fid == -1
    error('Cannot open file: %s\nCheck that DATA_FILE path is correct.', DATA_FILE);
end

rows = [];
foundHeader = false;

while ~feof(fid)
    line = strtrim(fgetl(fid));

    % Skip comment lines and blank lines
    if isempty(line) || line(1) == '#'
        continue
    end

    % Skip the CSV header row
    if ~foundHeader && startsWith(line, 't_ms')
        foundHeader = true;
        continue
    end

    % Parse numeric row
    vals = sscanf(line, '%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f');
    if numel(vals) == 15
        rows(end+1, :) = vals'; %#ok<AGROW>
    end
end
fclose(fid);

if isempty(rows)
    error('No data rows found. Check that the file contains CSV output from the simulation.');
end

t_ms    = rows(:, 1);
ref_x   = rows(:, 2);   ref_y   = rows(:, 3);
sim_x   = rows(:, 4);   sim_y   = rows(:, 5);
est_x   = rows(:, 6);   est_y   = rows(:, 7);
accel_x = rows(:, 8);   accel_y = rows(:, 9);
err_x   = rows(:,10);   err_y   = rows(:,11);
ref_vx  = rows(:,12);   ref_vy  = rows(:,13);
sim_vx  = rows(:,14);   sim_vy  = rows(:,15);

t_s = t_ms / 1000;   % convert to seconds for plots

fprintf('Loaded %d samples  (%.2f s)\n', numel(t_s), t_s(end));

% ----------------------------------------------------------------
%  Figure output folder
% ----------------------------------------------------------------
if ~exist(FIG_FOLDER, 'dir')
    mkdir(FIG_FOLDER);
end
timestamp = datestr(now, 'yyyymmdd_HHMMSS');

saveFig = @(fig, name) exportgraphics(fig, ...
    fullfile(FIG_FOLDER, sprintf('%s_%s.png', timestamp, name)), ...
    'Resolution', 150);

% ----------------------------------------------------------------
%  Figure 1 — Position tracking
% ----------------------------------------------------------------
fig1 = figure('Name', 'Position Tracking', 'NumberTitle', 'off');
tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

% -- X axis --
nexttile;
hold on;
plot(t_s, ref_x,  'b--', 'LineWidth', 1.5, 'DisplayName', 'Reference');
plot(t_s, sim_x,  'r-',  'LineWidth', 1.5, 'DisplayName', 'Plant (sim)');
plot(t_s, est_x,  'g:',  'LineWidth', 1.5, 'DisplayName', 'Kalman est.');
hold off;
xlabel('Time (s)');  ylabel('Position (mm)');
title('X Axis — Position');
legend('Location', 'best');
grid on;

% -- Y axis --
nexttile;
hold on;
plot(t_s, ref_y,  'b--', 'LineWidth', 1.5, 'DisplayName', 'Reference');
plot(t_s, sim_y,  'r-',  'LineWidth', 1.5, 'DisplayName', 'Plant (sim)');
plot(t_s, est_y,  'g:',  'LineWidth', 1.5, 'DisplayName', 'Kalman est.');
hold off;
xlabel('Time (s)');  ylabel('Position (mm)');
title('Y Axis — Position');
legend('Location', 'best');
grid on;

% -- Tracking error --
nexttile;
hold on;
plot(t_s, err_x, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Error X');
plot(t_s, err_y, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Error Y');
yline(0, 'k--', 'LineWidth', 0.8);
hold off;
xlabel('Time (s)');  ylabel('Error (mm)');
title('Tracking Error (ref - sim)');
legend('Location', 'best');
grid on;

% -- Acceleration command --
nexttile;
hold on;
plot(t_s, accel_x, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Accel X');
plot(t_s, accel_y, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Accel Y');
yline(0, 'k--', 'LineWidth', 0.8);
hold off;
xlabel('Time (s)');  ylabel('Acceleration (mm/s²)');
title('Control Output — Total Acceleration Command');
legend('Location', 'best');
grid on;

saveFig(fig1, '1_position_tracking');

% ----------------------------------------------------------------
%  Figure 2 — 2D path
% ----------------------------------------------------------------
fig2 = figure('Name', '2D Path', 'NumberTitle', 'off');
hold on;
plot(ref_x, ref_y, 'b--', 'LineWidth', 1.5, 'DisplayName', 'Reference path');
plot(sim_x, sim_y, 'r-',  'LineWidth', 1.5, 'DisplayName', 'Simulated path');
plot(ref_x(1),   ref_y(1),   'bs', 'MarkerSize', 10, 'MarkerFaceColor', 'b', 'HandleVisibility', 'off');
plot(ref_x(end), ref_y(end), 'b^', 'MarkerSize', 10, 'MarkerFaceColor', 'b', 'HandleVisibility', 'off');
hold off;
xlabel('X (mm)');  ylabel('Y (mm)');
title('2D Path — Reference vs Simulated Plant');
legend('Location', 'best');
axis equal;
grid on;

saveFig(fig2, '2_path_2d');

% ----------------------------------------------------------------
%  Figure 3 — Velocity tracking
%  ref_vx/vy come directly from the TrajectoryPlanner output.
%  sim_vx/vy come from the Euler-integrated plant state.
%  Both are output by the Teensy — no differentiation needed.
% ----------------------------------------------------------------

fig3 = figure('Name', 'Velocity Tracking', 'NumberTitle', 'off');
tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

nexttile;
hold on;
plot(t_s, ref_vx, 'b--', 'LineWidth', 1.5, 'DisplayName', 'Reference');
plot(t_s, sim_vx, 'r-',  'LineWidth', 1.5, 'DisplayName', 'Plant (sim)');
hold off;
xlabel('Time (s)');  ylabel('Velocity (mm/s)');
title('X Axis — Velocity');
legend('Location', 'best');  grid on;
% What to look for:
%   ref_vx should show a smooth trapezoidal (S-curve) profile.
%   sim_vx should closely follow ref_vx.
%   Persistent lag in sim_vx = Kd too weak.
%   sim_vx oscillating around ref_vx = Kp too high or Kd too low.

nexttile;
hold on;
plot(t_s, ref_vy, 'b--', 'LineWidth', 1.5, 'DisplayName', 'Reference');
plot(t_s, sim_vy, 'r-',  'LineWidth', 1.5, 'DisplayName', 'Plant (sim)');
hold off;
xlabel('Time (s)');  ylabel('Velocity (mm/s)');
title('Y Axis — Velocity');
legend('Location', 'best');  grid on;

saveFig(fig3, '3_velocity_tracking');

% ----------------------------------------------------------------
%  Figure 4 — Phase portrait (position vs velocity, per axis)
%  Shows the state-space trajectory of the controller.
%  Useful for spotting oscillation and overshoot before they are
%  obvious in the time-domain plots.
% ----------------------------------------------------------------
fig4 = figure('Name', 'Phase Portrait', 'NumberTitle', 'off');
tiledlayout(1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

nexttile;
hold on;
plot(ref_x, ref_vx, 'b--', 'LineWidth', 1.5, 'DisplayName', 'Reference');
plot(sim_x, sim_vx, 'r-',  'LineWidth', 1.5, 'DisplayName', 'Plant (sim)');
plot(sim_x(1),   sim_vx(1),   'rs', 'MarkerSize', 8, 'MarkerFaceColor', 'r', 'HandleVisibility', 'off');
plot(sim_x(end), sim_vx(end), 'r^', 'MarkerSize', 8, 'MarkerFaceColor', 'r', 'HandleVisibility', 'off');
hold off;
xlabel('Position (mm)');  ylabel('Velocity (mm/s)');
title('X Axis — Phase Portrait');
legend('Location', 'best');  grid on;
% What to look for:
%   A clean arc from (0,0) to (target,0) with no loops = well-damped.
%   Loops/spirals around the target = oscillation (Kd too low).
%   Sim arc extends past target x position = overshoot (Kp too high).

nexttile;
hold on;
plot(ref_y, ref_vy, 'b--', 'LineWidth', 1.5, 'DisplayName', 'Reference');
plot(sim_y, sim_vy, 'r-',  'LineWidth', 1.5, 'DisplayName', 'Plant (sim)');
plot(sim_y(1),   sim_vy(1),   'rs', 'MarkerSize', 8, 'MarkerFaceColor', 'r', 'HandleVisibility', 'off');
plot(sim_y(end), sim_vy(end), 'r^', 'MarkerSize', 8, 'MarkerFaceColor', 'r', 'HandleVisibility', 'off');
hold off;
xlabel('Position (mm)');  ylabel('Velocity (mm/s)');
title('Y Axis — Phase Portrait');
legend('Location', 'best');  grid on;

saveFig(fig4, '4_phase_portrait');

% ----------------------------------------------------------------
%  Figure 5 — Kalman innovation proxy
%  Innovation = measurement - predicted measurement = sim - est.
%  In simulation (no noise) this should decay quickly to ~0.
%  On hardware: if this has long-term structure (drift/oscillation)
%  rather than looking like white noise, Q is too small — the
%  filter is not tracking unexpected disturbances fast enough.
%  If it is very noisy/spiky, R is too small — the filter is
%  trusting the encoder too much.
% ----------------------------------------------------------------
innov_x = sim_x - est_x;
innov_y = sim_y - est_y;

fig5 = figure('Name', 'Kalman Innovation (sim − est)', 'NumberTitle', 'off');
tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

nexttile;
plot(t_s, innov_x, 'r-', 'LineWidth', 1.2);
yline(0, 'k--', 'LineWidth', 0.8);
xlabel('Time (s)');  ylabel('Innovation (mm)');
title('X — Innovation (sim − Kalman estimate)');
grid on;

nexttile;
plot(t_s, innov_y, 'b-', 'LineWidth', 1.2);
yline(0, 'k--', 'LineWidth', 0.8);
xlabel('Time (s)');  ylabel('Innovation (mm)');
title('Y — Innovation (sim − Kalman estimate)');
grid on;

saveFig(fig5, '5_kalman_innovation');

% ----------------------------------------------------------------
%  Figure 6 — Cumulative RMS tracking error
%  Single number summary of overall tracking performance.
%  Useful for comparing runs after changing gains or constants.
% ----------------------------------------------------------------
err_2d   = sqrt(err_x.^2 + err_y.^2);
rms_err  = sqrt(cumsum(err_2d.^2) ./ (1:numel(err_2d))');

fig6 = figure('Name', 'Cumulative RMS Error', 'NumberTitle', 'off');
plot(t_s, rms_err, 'k-', 'LineWidth', 1.5);
xlabel('Time (s)');  ylabel('RMS Error (mm)');
title('Cumulative RMS Tracking Error');
grid on;
% What to look for:
%   The curve should rise briefly during the acceleration phase
%   then flatten and ideally fall as the move settles.
%   The final value is your headline tracking performance number.
%   Use this to compare runs: lower final RMS = better tuning.

saveFig(fig6, '6_rms_error');

% ----------------------------------------------------------------
%  Console summary
% ----------------------------------------------------------------
peak_err  = max(err_2d);
final_err = err_2d(end);
final_rms = rms_err(end);

fprintf('\n--- Move summary ---\n');
fprintf('  Duration:          %.3f s\n',  t_s(end));
fprintf('  Peak 2D error:     %.4f mm\n', peak_err);
fprintf('  Final 2D error:    %.4f mm\n', final_err);
fprintf('  Final RMS error:   %.4f mm\n', final_rms);
fprintf('  Peak accel X:      %.2f mm/s²\n', max(abs(accel_x)));
fprintf('  Peak accel Y:      %.2f mm/s²\n', max(abs(accel_y)));
fprintf('\n--- Kalman innovation ---\n');
fprintf('  Max innovation X:  %.4f mm\n', max(abs(innov_x)));
fprintf('  Max innovation Y:  %.4f mm\n', max(abs(innov_y)));
fprintf('  Final innov X:     %.4f mm\n', innov_x(end));
fprintf('  Final innov Y:     %.4f mm\n', innov_y(end));
fprintf('\nFigures saved to: %s\n', FIG_FOLDER);
