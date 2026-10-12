function [u, mem] = ctrl_pid(x, d, mem, p) %#ok<INUSL>
%CTRL_PID  Split-range PID on coolant temperature (electric actuators).
%
%   One PID output y in [0, 3] is split across the actuators in order of
%   energy cost, so the cheapest one is used first:
%       0..1  thermostat valve 0 -> 100 %  (almost free)
%       1..2  pump 15 %  -> 100 %          (up to 1.5 kW)
%       2..3  fan   0 %  -> 100 %          (up to 25 kW)
%
%   Discrete PID, derivative on measurement with a first-order filter,
%   anti-windup by conditional integration (integrator frozen when the
%   output is saturated and the error would push it further).
g  = p.pid;
Tc = x(1);
e  = Tc - p.T_ref;                 % positive when too hot -> more cooling
if isempty(mem), mem.I = 0; mem.D = 0; mem.Tc_prev = Tc; end

% filtered derivative of the measurement
alpha = g.Tf / (g.Tf + g.Ts);
mem.D = alpha*mem.D + (1 - alpha) * g.Kd * (Tc - mem.Tc_prev) / g.Ts;
mem.Tc_prev = Tc;

y_unsat = g.Kp*e + mem.I + mem.D;
y = min(max(y_unsat, 0), 3);

% conditional integration (anti-windup)
if ~((y_unsat >= 3 && e > 0) || (y_unsat <= 0 && e < 0))
    mem.I = mem.I + g.Ki * e * g.Ts;
end
mem.I = min(max(mem.I, -1), 3);

xv = min(max(y, 0), 1);
up = p.u_min(2) + (1 - p.u_min(2)) * min(max(y - 1, 0), 1);
uf = min(max(y - 2, 0), 1);
u  = [uf; up; xv];
end
