function [dx, y] = thermal_model(x, u, d, p)
%THERMAL_MODEL  Nonlinear lumped thermal model of a truck engine cooling system.
%
%   [dx, y] = thermal_model(x, u, d, p)
%
%   States  x = [T_c; T_r; T_o]       (degC)
%       T_c  coolant / engine block temperature
%       T_r  radiator coolant temperature
%       T_o  engine oil temperature
%   Inputs  u = [u_f; u_p; x_v]       (0..1) manipulated variables
%       u_f  cooling-fan speed
%       u_p  coolant-pump speed
%       x_v  thermostat valve position (0 = full bypass, 1 = full radiator)
%   Disturbances d = [N; Tq; v; T_amb]
%       N rpm, Tq Nm, v km/h, T_amb degC
%
%   Outputs y = [P_fan; P_pump; mdot_fuel; Q_rad; Q_cool]
%       W, W, kg/s, W, W
%
%   Energy balances:
%     C_c dT_c/dt = Q_cool + Q_oc - mdot_r cp (T_c - T_r) - UA_amb (T_c - T_amb)
%     C_r dT_r/dt = mdot_r cp (T_c - T_r) - Q_rad
%     C_o dT_o/dt = Q_oil - Q_oc - UA_oamb (T_o - T_amb)
%   Nonlinearities: radiator effectiveness (eps-NTU), UA ~ air flow^0.8,
%   fan / pump power ~ speed^3, bilinear flow*temperature terms, friction
%   that rises with cold oil.

Tc = x(1); Tr = x(2); To = x(3);

uf = min(max(u(1), 0), 1);
up = min(max(u(2), 0), 1);
xv = min(max(u(3), 0), 1);

N  = max(d(1), 0);
Tq = max(d(2), 0);
v  = max(d(3), 0) / 3.6;     % km/h -> m/s
Ta = d(4);

% ---- Engine heat generation ----
P_b    = Tq * N * 2*pi/60;                         % brake power [W]
P_fan  = p.P_fan_max  * uf^3;                      % fan affinity law
P_pump = p.P_pump_max * up^3;                      % pump affinity law
P_fuel0 = (P_b + P_fan + P_pump) / p.eta_b + p.P_fuel_idle * N / p.N_idle;
fric    = p.k_fric * max(0, p.T_oil_warm - To);    % cold-oil friction penalty
P_fuel  = P_fuel0 * (1 + fric);

Q_cool = p.frac_cool * P_fuel;                     % into coolant jacket
Q_oil  = p.frac_oil  * P_fuel + fric * P_fuel0;    % extra friction heats oil

% ---- Coolant flows ----
mdot   = p.mdot_max * up;
mdot_r = xv * mdot;                                % through radiator

% ---- Radiator (eps-NTU, air side limited) ----
mdot_air = p.rho_a * p.A_front * p.k_ram * v + p.mdot_fan_max * uf + 0.05;
UA_rad   = p.UA_rad0 * (mdot_air / p.mdot_air_ref)^0.8;
C_air    = mdot_air * p.cp_a;
eff      = 1 - exp(-UA_rad / C_air);
Q_rad    = eff * C_air * (Tr - Ta);

% ---- Oil cooler ----
Q_oc = p.UA_oc0 * (0.3 + 0.7*up) * (To - Tc);

% ---- State derivatives ----
dTc = (Q_cool + Q_oc - mdot_r*p.cp_c*(Tc - Tr) - p.UA_amb*(Tc - Ta)) / p.C_c;
dTr = (mdot_r*p.cp_c*(Tc - Tr) - Q_rad) / p.C_r;
dTo = (Q_oil - Q_oc - p.UA_oamb*(To - Ta)) / p.C_o;

dx = [dTc; dTr; dTo];
y  = [P_fan; P_pump; P_fuel / p.LHV; Q_rad; Q_cool];
end
