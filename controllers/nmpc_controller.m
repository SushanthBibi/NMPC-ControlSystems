function [u, mem, info] = nmpc_controller(x, t, mem, p, dfun)
%NMPC_CONTROLLER  Nonlinear model predictive controller for engine cooling.
%
%   [u, mem, info] = nmpc_controller(x, t, mem, p, dfun)
%     x     measured state [T_c; T_r; T_o]
%     t     current time [s]
%     mem   controller memory ([] on first call)
%     p     parameters (thermal_params), uses p.mpc
%     dfun  handle returning future disturbances d(t) (preview of speed /
%           load); if p.mpc.preview is false the current d is held.
%
%   At every sample the controller
%     1. predicts the states over Np steps with the nonlinear model,
%     2. minimises
%          J = sum_k  w_T H(T_c,k - T_ref)            temperature tracking
%                   + w_fuel * fuel_k [g/s]           fuel incl. fan/pump
%                   + sum_i w_du,i (du_i)^2           smooth actuation
%                   + w_soft * (soft limit excess)^2  T_c and T_o limits
%        over the future inputs (move blocking: few free moves),
%        H(e) = e^2 for |e| <= huber, linear beyond (Huber penalty) so that
%        a cold engine is not "heated" by burning fuel on fan load,
%     3. applies only the first input and repeats (receding horizon).
%
%   Input bounds are handled exactly: fmincon with lb/ub when the
%   Optimization Toolbox is present, otherwise fminsearch (base MATLAB)
%   with a sine transform that maps any real number into [u_min, u_max].

m  = p.mpc;
nu = 3;
nb = numel(m.blocks);
if isempty(mem)
    mem.U      = repmat([0; 0.4; 0.5], 1, nb);   % initial guess
    mem.u_prev = [0; 0.4; 0.5];
    mem.use_fmincon = exist('fmincon', 'file') == 2 && ...
        ~exist('OCTAVE_VERSION', 'builtin');
end

% ---- disturbance preview over the horizon ----
tk = t + (0:m.Np-1) * m.Ts;
if m.preview
    D = dfun(tk);
else
    D = repmat(dfun(t), 1, m.Np);
end

% ---- warm start: shift previous solution by one block ----
U0 = [mem.U(:, 2:end), mem.U(:, end)];
lb = repmat(p.u_min, nb, 1);
ub = repmat(p.u_max, nb, 1);
cost = @(z) nmpc_cost(reshape(z, nu, nb), x, D, mem.u_prev, p);

if mem.use_fmincon
    opts = optimoptions('fmincon', 'Display', 'off', 'Algorithm', 'sqp', ...
        'MaxIterations', m.maxIter, 'OptimalityTolerance', 1e-4);
    [z, J, flag] = fmincon(cost, U0(:), [], [], [], [], lb, ub, [], opts);
else
    % bounded -> unbounded transform for fminsearch
    to_u = @(s) lb + (ub - lb) .* (0.5 + 0.5*sin(s));
    r    = min(max((U0(:) - lb) ./ (ub - lb), 0), 1);
    s0   = asin(2*r - 1);
    opts = optimset('Display', 'off', 'MaxFunEvals', m.maxIter * 2, ...
        'MaxIter', m.maxIter, 'TolX', 1e-3, 'TolFun', 1e-3);
    [s, J, flag] = fminsearch(@(s) cost(to_u(s)), s0, opts);
    z = to_u(s);
end

U = reshape(z, nu, nb);
u = U(:, 1);
mem.U = U;
mem.u_prev = u;
info.J = J; info.flag = flag;
end

function J = nmpc_cost(U, x, D, u_prev, p)
% Predict over the horizon with move blocking and evaluate the cost.
m  = p.mpc;
J  = 0;
k  = 0;
up = u_prev;
for b = 1:numel(m.blocks)
    u  = U(:, b);
    du = u - up;
    J  = J + sum(m.w_du .* du.^2);
    up = u;
    for j = 1:m.blocks(b)
        k = k + 1;
        [x, y] = rk4_step(x, u, D(:, k), m.Ts, p);
        eT  = abs(x(1) - p.T_ref);
        if eT > m.huber                   % linear far from T_ref (Huber):
            eT2 = m.huber*(2*eT - m.huber); % stops "burn fuel to warm up"
        else
            eT2 = eT^2;
        end
        vc  = max(0, x(1) - (p.T_c_max - m.margin));
        vo  = max(0, x(3) - (p.T_o_max - m.margin));
        J = J + m.w_T * eT2 + m.w_fuel * 1e3 * y(3) + m.w_soft * (vc^2 + vo^2);
    end
end
end
