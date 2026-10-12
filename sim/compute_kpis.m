function k = compute_kpis(out, p)
%COMPUTE_KPIS  Performance indicators listed in the project methodology.
t  = out.t; dt = t(2) - t(1);
Tc = out.x(1, :); To = out.x(3, :);
warm = Tc >= p.T_warm;
idx  = find(warm, 1);
if isempty(idx), k.warmup_time = NaN; else, k.warmup_time = t(idx); end
if isempty(idx), idx = numel(t); end

k.Tc_std   = std(Tc(idx:end));               % coolant stability (after warm-up)
k.To_std   = std(To(idx:end));               % oil stability
k.Tc_rmse  = sqrt(mean((Tc(idx:end) - p.T_ref).^2));
k.Tc_peak  = max(Tc);
k.To_peak  = max(To);
k.E_fan    = sum(out.y(1, :)) * dt / 3.6e6;  % kWh
k.E_pump   = sum(out.y(2, :)) * dt / 3.6e6;  % kWh
k.E_cool   = k.E_fan + k.E_pump;             % kWh
k.fuel     = sum(out.y(3, :)) * dt;          % kg
end
