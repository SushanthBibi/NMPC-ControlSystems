function u = sl_split_range(y)
%SL_SPLIT_RANGE  Map PID output y in [0,3] to [fan; pump; valve] (see ctrl_pid).
persistent p
if isempty(p), p = thermal_params(); end
xv = min(max(y, 0), 1);
up = p.u_min(2) + (1 - p.u_min(2)) * min(max(y - 1, 0), 1);
uf = min(max(y - 2, 0), 1);
u  = [uf; up; xv];
end
