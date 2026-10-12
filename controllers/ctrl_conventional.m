function [u, mem] = ctrl_conventional(x, d, mem, p)
%CTRL_CONVENTIONAL  Today's truck cooling: wax thermostat, belt pump, fan clutch.
%
%   - Thermostat: wax element opens linearly between T_open_start and
%     T_open_full, with a first-order lag (time constant tau_wax).
%   - Pump: belt driven, speed proportional to engine speed.
%   - Fan: on/off viscous clutch with hysteresis (T_fan_on / T_fan_off);
%     when engaged it turns with the engine.
c  = p.conv;
Tc = x(1); N = d(1);
if isempty(mem), mem.xv = 0; mem.fan_on = false; end

% wax thermostat (first-order lag towards the static opening curve)
xv_ss  = min(max((Tc - c.T_open_start) / (c.T_open_full - c.T_open_start), 0), 1);
mem.xv = mem.xv + p.dt / c.tau_wax * (xv_ss - mem.xv);

% fan clutch with hysteresis
if Tc >= c.T_fan_on,  mem.fan_on = true;  end
if Tc <= c.T_fan_off, mem.fan_on = false; end
uf = mem.fan_on * min(1, N / p.N_rated);

% mechanical pump
up = min(max(N / c.N_pump_full, p.u_min(2)), 1);

u = [uf; up; mem.xv];
end
