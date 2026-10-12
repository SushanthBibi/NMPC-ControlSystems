function res = validate_model(csv_file)
%VALIDATE_MODEL  Calibrate and validate the thermal model (RMSE and R^2).
%
%   res = validate_model()            uses synthetic reference data
%   res = validate_model('data.csv')  uses measured data
%
%   CSV columns (header row, one sample per row, constant time step):
%     t, N, Tq, v, T_amb, u_f, u_p, x_v, T_c, T_o
%
%   Steps
%     1. Simulate the model open-loop with the recorded inputs.
%     2. Calibrate three scale factors (coolant heat, radiator UA, oil heat)
%        with fminsearch on the first half of the data (identification).
%     3. Report RMSE and R^2 of T_c and T_o on the second half (validation).
%
%   NOTE: With no CSV, the "reference" is generated from the same model
%   with deliberately wrong parameters (+12 % coolant heat, -15 % radiator
%   UA, +10 % oil heat) and sensor noise. This only demonstrates the
%   workflow; replace it with test-bench or vehicle data to really
%   validate the model.
p = thermal_params();

if nargin < 1 || isempty(csv_file)
    [ref, src] = synthetic_reference(p);
else
    A   = dlmread(csv_file, ',', 1, 0);
    ref = struct('t', A(:,1).', 'd', A(:,2:5).', 'u', A(:,6:8).', ...
                 'Tc', A(:,9).', 'To', A(:,10).');
    src = csv_file;
end
n   = numel(ref.t);
idI = 1:floor(n/2);           % identification half
idV = floor(n/2)+1:n;         % validation half
x0  = [ref.Tc(1); ref.Tc(1); ref.To(1)];

% ---- before calibration ----
[Tc0, To0] = sim_open_loop(p, ref, x0);

% ---- calibrate 3 scale factors on the identification half ----
cost = @(th) calib_cost(th, p, ref, x0, idI);
th   = fminsearch(cost, [1; 1; 1], optimset('Display', 'off', 'TolX', 1e-3));
pc   = apply_scale(p, th);
[Tc1, To1] = sim_open_loop(pc, ref, x0);

res.source = src;
res.theta  = th;
res.before = fit_stats(ref, Tc0, To0, idV);
res.after  = fit_stats(ref, Tc1, To1, idV);
res.p_calibrated = pc;

fprintf('\nModel validation  (%s)\n', src);
fprintf('  calibrated scales: Q_cool x%.3f  UA_rad x%.3f  Q_oil x%.3f\n', th);
fprintf('  %-18s %10s %10s %10s %10s\n', '', 'RMSE Tc', 'R2 Tc', 'RMSE To', 'R2 To');
fprintf('  %-18s %10.2f %10.4f %10.2f %10.4f\n', 'nominal model', ...
    res.before.rmse_Tc, res.before.r2_Tc, res.before.rmse_To, res.before.r2_To);
fprintf('  %-18s %10.2f %10.4f %10.2f %10.4f\n', 'calibrated model', ...
    res.after.rmse_Tc, res.after.r2_Tc, res.after.rmse_To, res.after.r2_To);

figure('Name', 'Model validation');
subplot(2,1,1); plot(ref.t, ref.Tc, 'k.', ref.t, Tc0, 'b--', ref.t, Tc1, 'r-');
xline_compat(ref.t(idV(1))); ylabel('T_c [degC]'); grid on;
legend('reference', 'nominal model', 'calibrated model', 'Location', 'best');
title('Coolant temperature (left: identification, right: validation)');
subplot(2,1,2); plot(ref.t, ref.To, 'k.', ref.t, To0, 'b--', ref.t, To1, 'r-');
xline_compat(ref.t(idV(1))); ylabel('T_o [degC]'); xlabel('time [s]'); grid on;
end

% ------------------------------------------------------------------------
function [ref, src] = synthetic_reference(p)
pt = p;                                   % "true" plant, unknown to us
pt.frac_cool = p.frac_cool * 1.12;
pt.UA_rad0   = p.UA_rad0   * 0.85;
pt.frac_oil  = p.frac_oil  * 1.10;
cyc = drive_cycle('hill');
t   = 0:p.dt:cyc.T_end;
d   = cyc.d(t);
% open-loop excitation of the actuators (steps / ramps)
uf  = 0.3 + 0.3*(mod(floor(t/240), 3) == 1) + 0.4*(mod(floor(t/240), 3) == 2);
up  = 0.4 + 0.4*(mod(floor(t/170), 2) == 1);
xv  = min(1, 0.4 + 0.6*(mod(floor(t/300), 2) == 1));
ref.t = t; ref.d = d; ref.u = [uf; up; xv];
x = cyc.x0; X = zeros(3, numel(t));
for k = 1:numel(t)
    X(:, k) = x;
    x = rk4_step(x, ref.u(:, k), d(:, k), p.dt, pt);
end
randn('seed', 1); %#ok<RAND>
ref.Tc = X(1, :) + 0.3*randn(1, numel(t));    % sensor noise
ref.To = X(3, :) + 0.3*randn(1, numel(t));
src = 'synthetic reference (replace with measured CSV)';
end

function [Tc, To] = sim_open_loop(p, ref, x0)
n = numel(ref.t); x = x0; X = zeros(3, n);
dt = ref.t(2) - ref.t(1);
for k = 1:n
    X(:, k) = x;
    x = rk4_step(x, ref.u(:, k), ref.d(:, k), dt, p);
end
Tc = X(1, :); To = X(3, :);
end

function J = calib_cost(th, p, ref, x0, id)
if any(th <= 0.2) || any(th > 5), J = 1e9; return; end
[Tc, To] = sim_open_loop(apply_scale(p, th), ref, x0);
J = mean((Tc(id) - ref.Tc(id)).^2) + mean((To(id) - ref.To(id)).^2);
end

function p = apply_scale(p, th)
p.frac_cool = p.frac_cool * th(1);
p.UA_rad0   = p.UA_rad0   * th(2);
p.frac_oil  = p.frac_oil  * th(3);
end

function s = fit_stats(ref, Tc, To, id)
s.rmse_Tc = sqrt(mean((Tc(id) - ref.Tc(id)).^2));
s.rmse_To = sqrt(mean((To(id) - ref.To(id)).^2));
s.r2_Tc   = 1 - sum((Tc(id) - ref.Tc(id)).^2) / sum((ref.Tc(id) - mean(ref.Tc(id))).^2);
s.r2_To   = 1 - sum((To(id) - ref.To(id)).^2) / sum((ref.To(id) - mean(ref.To(id))).^2);
end

function xline_compat(x)
yl = ylim; hold on; plot([x x], yl, 'k:'); hold off;
end
