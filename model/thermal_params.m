function p = thermal_params()
%THERMAL_PARAMS  Parameters of the lumped heavy-duty truck cooling model.
%
%   p = thermal_params() returns a struct with every constant used by the
%   plant model, the controllers and the simulations. Values are typical
%   for a 13 L class heavy-duty diesel (about 400 kW rated). Replace them
%   with measured values once test data is available (see validate_model).

% ---------------- Engine ----------------
p.N_idle      = 600;      % idle speed                    [rpm]
p.N_rated     = 1800;     % rated speed                   [rpm]
p.Tq_max      = 2600;     % maximum torque                [Nm]
p.eta_b       = 0.42;     % brake thermal efficiency (warm engine) [-]
p.P_fuel_idle = 15e3;     % fuel power to overcome friction at idle [W]
p.LHV         = 42.8e6;   % diesel lower heating value    [J/kg]
p.frac_cool   = 0.22;     % share of fuel power into coolant jacket [-]
p.frac_oil    = 0.04;     % share of fuel power into engine oil     [-]
p.k_fric      = 0.0015;   % extra fuel per K of oil below T_oil_warm [1/K]
p.T_oil_warm  = 90;       % oil temperature with no friction penalty [degC]

% ---------------- Thermal masses ----------------
p.C_c = 200e3;            % engine block + jacket coolant [J/K]
p.C_r = 90e3;             % radiator + hoses coolant      [J/K]
p.C_o = 100e3;            % oil sump + oil                [J/K]

% ---------------- Coolant circuit ----------------
p.cp_c     = 3600;        % 50/50 water-glycol            [J/(kg K)]
p.mdot_max = 5.0;         % pump flow at full speed       [kg/s]

% ---------------- Radiator (air side) ----------------
p.cp_a         = 1005;    % air specific heat             [J/(kg K)]
p.rho_a        = 1.15;    % air density                   [kg/m^3]
p.A_front      = 1.0;     % radiator frontal area         [m^2]
p.k_ram        = 0.30;    % ram-air recovery factor       [-]
p.mdot_fan_max = 8.0;     % air flow from fan at 100 %    [kg/s]
p.UA_rad0      = 6.0e3;   % radiator UA at reference air flow [W/K]
p.mdot_air_ref = 8.0;     % reference air flow            [kg/s]

% ---------------- Oil cooler and losses ----------------
p.UA_oc0  = 2.0e3;        % oil-to-coolant UA at full pump flow [W/K]
p.UA_amb  = 0.3e3;        % engine block to ambient       [W/K]
p.UA_oamb = 0.1e3;        % oil sump to ambient           [W/K]

% ---------------- Actuators (manipulated variables) ----------------
%   u = [fan speed; pump speed; thermostat valve position], all 0..1
p.P_fan_max  = 25e3;      % fan shaft power at 100 %      [W]
p.P_pump_max = 1.5e3;     % pump power at 100 %           [W]
p.u_min = [0; 0.15; 0];   % pump never fully stops
p.u_max = [1; 1; 1];

% ---------------- Targets and limits ----------------
p.T_ref   = 90;           % coolant temperature set-point [degC]
p.T_c_max = 105;          % coolant upper limit           [degC]
p.T_o_max = 125;          % oil upper limit               [degC]
p.T_warm  = 80;           % "warmed up" coolant threshold [degC]

% ---------------- Simulation ----------------
p.dt = 1;                 % plant integration step        [s]

% ---------------- Conventional controller ----------------
p.conv.T_open_start = 82; % wax thermostat starts to open [degC]
p.conv.T_open_full  = 95; % wax thermostat fully open     [degC]
p.conv.tau_wax      = 20; % wax element time constant     [s]
p.conv.T_fan_on     = 96; % fan clutch engages            [degC]
p.conv.T_fan_off    = 90; % fan clutch releases           [degC]
p.conv.N_pump_full  = 2000; % engine speed giving 100 % pump [rpm]

% ---------------- PID controller ----------------
p.pid.Kp  = 0.30;         % [1/K]
p.pid.Ki  = 0.006;        % [1/(K s)]
p.pid.Kd  = 2.0;          % [s/K]
p.pid.Tf  = 10;           % derivative filter time constant [s]
p.pid.Ts  = 1;            % sample time [s]
p.pid.antiwindup = true;  % conditional-integration anti-windup

% ---------------- NMPC ----------------
p.mpc.Ts      = 5;        % controller sample time = prediction step [s]
p.mpc.Np      = 12;       % prediction horizon (12*5 s = 60 s)
p.mpc.blocks  = [2 10];   % move blocking: input held constant per block
p.mpc.w_T     = 0.2;      % weight on (T_c - T_ref)^2          [1/K^2]
p.mpc.huber   = 5;        % tracking cost is linear beyond +-5 K   [K]
p.mpc.w_fuel  = 10;       % weight on fuel rate                [1/(g/s)]
p.mpc.w_du    = [2; 2; 2];% weight on input changes
p.mpc.w_soft  = 1e3;      % weight on soft constraint violation
p.mpc.margin  = 2;        % keep this far below T_c_max / T_o_max [K]
p.mpc.preview = true;     % use predicted speed / load over horizon
p.mpc.maxIter = 300;      % optimiser iteration budget
end
