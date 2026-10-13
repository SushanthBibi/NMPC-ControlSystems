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
