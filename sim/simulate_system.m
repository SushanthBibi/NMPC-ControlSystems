function out = simulate_system(ctrl_name, scenario, p)
%SIMULATE_SYSTEM  Closed-loop simulation of plant + chosen controller.
%
%   out = simulate_system(ctrl_name, scenario)
%     ctrl_name  'conventional' | 'pid' | 'nmpc'
%     scenario   name accepted by drive_cycle, or a cycle struct
%
%   The plant is integrated with RK4 at p.dt. Conventional and PID run
%   every p.dt, the NMPC every p.mpc.Ts (zero-order hold in between).
if nargin < 3, p = thermal_params(); end
if ischar(scenario), cyc = drive_cycle(scenario); else, cyc = scenario; end

t  = 0:p.dt:cyc.T_end;
n  = numel(t);
X  = zeros(3, n); U = zeros(3, n); Y = zeros(5, n); Dd = zeros(4, n);
x  = cyc.x0; mem = []; u = [0; p.u_min(2); 0];
n_ctrl = 1;
if strcmpi(ctrl_name, 'nmpc'), n_ctrl = round(p.mpc.Ts / p.dt); end

tic;
for k = 1:n
    d = cyc.d(t(k));
    if mod(k-1, n_ctrl) == 0
        switch lower(ctrl_name)
            case 'conventional', [u, mem] = ctrl_conventional(x, d, mem, p);
            case 'pid',          [u, mem] = ctrl_pid(x, d, mem, p);
            case 'nmpc',         [u, mem] = nmpc_controller(x, t(k), mem, p, cyc.d);
            otherwise, error('unknown controller "%s"', ctrl_name);
        end
    end
    [x_next, y] = rk4_step(x, u, d, p.dt, p);
    X(:, k) = x; U(:, k) = u; Y(:, k) = y; Dd(:, k) = d;
    x = x_next;
end

out.name = lower(ctrl_name); out.scenario = cyc.name;
out.t = t; out.x = X; out.u = U; out.y = Y; out.d = Dd;
out.cpu_time = toc;
out.kpi = compute_kpis(out, p);
end
