%RUN_PID_REPORT  PID-only experiments and report figures.
%
%   Runs five classic control-systems experiments on the engine cooling
%   model and saves every figure as a PNG in the folder figures/.
%
%     1. Load step test   : P vs PI vs PID (offset, overshoot, settling)
%     2. P / I / D terms  : what each term contributes during the step
%     3. Set-point step   : 90 -> 95 -> 90 degC tracking
%     4. Anti-windup      : cold start with and without anti-windup
%     5. PID vs today's   : PID vs conventional thermostat on 4 scenarios
%
%   Run time: under a minute. Each section (%%) can also be run on its own
%   with "Run Section" once the first section has run.

%% 0. Setup
clear; clc; close all;
root = fileparts(mfilename('fullpath'));
addpath(fullfile(root, 'model'), fullfile(root, 'controllers'), fullfile(root, 'sim'));
figdir = fullfile(root, 'figures');
if ~exist(figdir, 'dir'), mkdir(figdir); end
p0 = thermal_params();

%% 1. Load step test: P vs PI vs PID
% Engine load jumps 30 % -> 80 % at t = 600 s and back at t = 1500 s.
cases = { 'P only', p0.pid.Kp, 0,          0;
          'PI',     p0.pid.Kp, p0.pid.Ki,  0;
          'PID',    p0.pid.Kp, p0.pid.Ki,  p0.pid.Kd };
res1 = cell(1, 3);
for i = 1:3
    p = p0;
    p.pid.Kp = cases{i, 2}; p.pid.Ki = cases{i, 3}; p.pid.Kd = cases{i, 4};
    res1{i} = pid_run(p, 'step', []);
end

fig = figure('Name', 'Load step: P vs PI vs PID', 'Position', [100 100 900 650]);
subplot(3,1,1); hold on;
for i = 1:3, plot(res1{i}.t, res1{i}.Tc, 'LineWidth', 1.3); end
plot(res1{1}.t, res1{1}.Tref, 'k--');
ylabel('T_c [\circC]'); grid on; ylim([80 100]);
legend('P only', 'PI', 'PID', 'set-point', 'Location', 'southeast');
title('Coolant temperature response to an engine-load step');
subplot(3,1,2); hold on;
for i = 1:3, plot(res1{i}.t, res1{i}.y, 'LineWidth', 1.3); end
ylabel('PID output y [-]'); grid on;
title('Controller output (0-1 valve, 1-2 pump, 2-3 fan)');
subplot(3,1,3);
plot(res1{1}.t, res1{1}.load*100, 'k', 'LineWidth', 1.3);
ylabel('Engine load [%]'); xlabel('time [s]'); grid on; ylim([0 100]);
title('Disturbance: engine load');
save_fig(fig, figdir, '1_load_step_P_PI_PID');

fprintf('\n=== Experiment 1: load step 30 %% -> 80 %% at t = 600 s ===\n');
fprintf('%-8s %14s %16s %18s\n', 'ctrl', 'peak dev [K]', 'settling [s]', 'steady err [K]');
for i = 1:3
    m = step_metrics(res1{i}, 600, 1500);
    fprintf('%-8s %14.2f %16.0f %18.2f\n', cases{i, 1}, m.peak, m.ts, m.ess);
end

%% 2. What each PID term does (PID case of experiment 1)
r = res1{3};
fig = figure('Name', 'PID terms', 'Position', [100 100 900 500]);
subplot(2,1,1);
plot(r.t, r.e, 'k', 'LineWidth', 1.3); grid on;
ylabel('error e = T_c - T_{ref} [K]'); title('Error signal');
subplot(2,1,2); hold on;
plot(r.t, r.P, 'LineWidth', 1.3); plot(r.t, r.I, 'LineWidth', 1.3);
plot(r.t, r.D, 'LineWidth', 1.3); plot(r.t, r.y, 'k--', 'LineWidth', 1);
grid on; ylabel('contribution [-]'); xlabel('time [s]');
legend('P = K_p e', 'I = K_i \int e dt', 'D = K_d de/dt (filtered)', 'total y', ...
    'Location', 'northeast');
title('Contribution of the P, I and D terms');
save_fig(fig, figdir, '2_PID_terms');

%% 3. Set-point tracking: 90 -> 95 -> 90 degC
Tref_fun = @(t) 90 + 5*(t >= 600 & t < 1500);
r3 = pid_run(p0, 'step', Tref_fun);
fig = figure('Name', 'Set-point step', 'Position', [100 100 900 450]);
subplot(2,1,1); hold on;
plot(r3.t, r3.Tc, 'LineWidth', 1.3); plot(r3.t, r3.Tref, 'k--');
ylabel('T_c [\circC]'); grid on; legend('T_c', 'set-point', 'Location', 'southeast');
title('Set-point tracking (load also steps at 600 s and 1500 s)');
subplot(2,1,2); hold on;
plot(r3.t, r3.u(1,:), r3.t, r3.u(2,:), r3.t, r3.u(3,:), 'LineWidth', 1.3);
ylabel('actuator [-]'); xlabel('time [s]'); grid on; ylim([-0.05 1.05]);
legend('fan', 'pump', 'valve', 'Location', 'northeast');
save_fig(fig, figdir, '3_setpoint_step');

%% 4. Integrator windup: cold start with and without anti-windup
pa = p0; pa.pid.antiwindup = true;
pb = p0; pb.pid.antiwindup = false;
r4a = pid_run(pa, 'coldstart', []);
r4b = pid_run(pb, 'coldstart', []);
fig = figure('Name', 'Anti-windup', 'Position', [100 100 900 500]);
subplot(2,1,1); hold on;
plot(r4b.t, r4b.Tc, 'r', 'LineWidth', 1.3); plot(r4a.t, r4a.Tc, 'b', 'LineWidth', 1.3);
plot(r4a.t, r4a.Tref, 'k--'); grid on; ylabel('T_c [\circC]');
legend('without anti-windup', 'with anti-windup', 'set-point', 'Location', 'southeast');
title('Cold start from 5 \circC');
subplot(2,1,2); hold on;
plot(r4b.t, r4b.I, 'r', 'LineWidth', 1.3); plot(r4a.t, r4a.I, 'b', 'LineWidth', 1.3);
grid on; ylabel('integral term I [-]'); xlabel('time [s]');
title('Integral term: winds up while the actuators are saturated');
save_fig(fig, figdir, '4_anti_windup');
fprintf('\n=== Experiment 4: cold start ===\n');
fprintf('peak T_c without anti-windup: %.1f degC, with: %.1f degC\n', max(r4b.Tc), max(r4a.Tc));

%% 5. PID vs conventional cooling on 4 driving scenarios
scen = {'highway', 'hill', 'urban', 'coldstart'};
K = zeros(numel(scen), 2, 5);
fig = figure('Name', 'PID vs conventional', 'Position', [100 100 1000 700]);
for i = 1:numel(scen)
    oc = simulate_system('conventional', scen{i}, p0);
    op = simulate_system('pid',          scen{i}, p0);
    subplot(2,2,i); hold on;
    plot(oc.t, oc.x(1,:), 'Color', [0.85 0.33 0.10], 'LineWidth', 1.2);
    plot(op.t, op.x(1,:), 'Color', [0 0.45 0.74], 'LineWidth', 1.2);
    plot(op.t([1 end]), p0.T_ref*[1 1], 'k--');
    grid on; ylabel('T_c [\circC]'); xlabel('time [s]'); title(scen{i});
    if i == 1, legend('conventional', 'PID', 'set-point', 'Location', 'southeast'); end
    if ~strcmp(scen{i}, 'coldstart'), ylim([75 100]); end
    for j = 1:2
        if j == 1, k = oc.kpi; else, k = op.kpi; end
        K(i, j, :) = [k.Tc_std, k.Tc_rmse, k.Tc_peak, k.E_fan + k.E_pump, k.fuel];
    end
end
save_fig(fig, figdir, '5_PID_vs_conventional');

fprintf('\n=== Experiment 5: PID vs conventional ===\n');
fprintf('%-10s %-13s %9s %9s %9s %12s %9s\n', 'scenario', 'controller', ...
    'Tc std', 'Tc rmse', 'Tc peak', 'fan+pump', 'fuel');
fprintf('%-10s %-13s %9s %9s %9s %12s %9s\n', '', '', '[K]', '[K]', '[degC]', '[kWh]', '[kg]');
names = {'conventional', 'PID'};
for i = 1:numel(scen)
    for j = 1:2
        fprintf('%-10s %-13s %9.2f %9.2f %9.1f %12.3f %9.3f\n', scen{i}, names{j}, squeeze(K(i, j, :)));
    end
end

fig = figure('Name', 'Temperature error RMSE', 'Position', [100 100 700 400]);
bar(squeeze(K(:, :, 2)));
set(gca, 'XTickLabel', scen); grid on;
ylabel('RMSE of T_c from 90 \circC [K]');
legend('conventional', 'PID', 'Location', 'northwest');
title('Temperature regulation error');
save_fig(fig, figdir, '6_rmse_bar');

fprintf('\nFigures saved in %s\n', figdir);

% ------------------------------------------------------------------------
% Local helper functions (kept in this file so the script is self-contained)

function r = pid_run(p, scenario, Tref_fun)
% Closed-loop PID simulation that also logs the P, I and D terms.
cyc = drive_cycle(scenario);
t = 0:p.dt:cyc.T_end; n = numel(t);
x = cyc.x0; mem = [];
r.t = t; r.Tc = zeros(1, n); r.Tref = zeros(1, n); r.u = zeros(3, n);
r.e = zeros(1, n); r.P = r.e; r.I = r.e; r.D = r.e; r.y = r.e; r.load = r.e;
for k = 1:n
    if ~isempty(Tref_fun), p.T_ref = Tref_fun(t(k)); end
    d = cyc.d(t(k));
    [u, mem] = ctrl_pid(x, d, mem, p);
    r.Tc(k) = x(1); r.Tref(k) = p.T_ref; r.u(:, k) = u; r.e(k) = x(1) - p.T_ref;
    r.P(k) = mem.P; r.I(k) = mem.I; r.D(k) = mem.D; r.y(k) = mem.y;
    r.load(k) = d(2) / p.Tq_max;
    x = rk4_step(x, u, d, p.dt, p);
end
end

function m = step_metrics(r, t_step, t_end)
% Peak deviation, 1 K settling time and steady-state error after a step.
idx = r.t >= t_step & r.t < t_end;
e   = r.Tc(idx) - r.Tref(idx);
tt  = r.t(idx);
[~, ip] = max(abs(e)); m.peak = e(ip);
out = find(abs(e) > 1, 1, 'last');
if isempty(out), m.ts = 0; else, m.ts = tt(out) - t_step; end
m.ess = mean(e(tt > t_end - 120));
end

function save_fig(fig, figdir, name)
print(fig, fullfile(figdir, [name '.png']), '-dpng', '-r150');
end
