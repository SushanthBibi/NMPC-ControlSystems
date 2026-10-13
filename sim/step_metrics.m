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
