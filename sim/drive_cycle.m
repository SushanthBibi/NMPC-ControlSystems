function cyc = drive_cycle(name)
%DRIVE_CYCLE  Representative operating scenarios for the cooling system.
%
%   cyc = drive_cycle(name), name is one of
%     'highway'    rolling-hill motorway cruise, 25 degC
%     'hill'       long full-load climb on a hot day, 38 degC
%     'urban'      stop-and-go city delivery, 30 degC
%     'coldstart'  warm-up from 5 degC soak
%
%   cyc.d(t) returns the disturbance vector [N; Tq; v; T_amb] for a scalar
%   or row vector t (one column per time). cyc.d is also what the NMPC
%   uses as its preview of future speed and load.

p = thermal_params();
switch lower(name)
    case 'highway'
        % time   speed  rpm   load
        tb = [0    60    1800];
        vb = [0    80    80  ];
        Nb = [700  1300  1300];
        Lb = [0.6  0.35  0.35];
        Ta = 25;  x0 = [88; 55; 92];  T_end = 1800;
        Lfun = @(t, L) L + 0.20*sin(2*pi*t/300);     % rolling hills
    case 'hill'
        tb = [0    500   560   1160  1200  1500  1540  1800];
        vb = [80   80    35    35    60    60    80    80  ];
        Nb = [1300 1300  1600  1600  1400  1400  1300  1300];
        Lb = [0.40 0.40  0.85  0.85  0.0   0.0   0.40  0.40];
        Ta = 38;  x0 = [90; 65; 95];  T_end = 1800;
        Lfun = @(t, L) L;
    case 'urban'
        % one 150 s stop-go block repeated
        tb1 = [0   30  50   110  125  150];
        vb1 = [0   0   40   40   0    0  ];
        Nb1 = [600 600 1400 1100 600  600];
        Lb1 = [0.05 0.05 0.70 0.25 0.0 0.05];
        nrep = 12; tb = []; vb = []; Nb = []; Lb = [];
        for k = 0:nrep-1
            tb = [tb, tb1(1:end-1) + 150*k]; %#ok<AGROW>
            vb = [vb, vb1(1:end-1)]; Nb = [Nb, Nb1(1:end-1)]; Lb = [Lb, Lb1(1:end-1)]; %#ok<AGROW>
        end
        tb = [tb, 150*nrep]; vb = [vb, 0]; Nb = [Nb, 600]; Lb = [Lb, 0.05];
        Ta = 30;  x0 = [88; 60; 92];  T_end = 1800;
        Lfun = @(t, L) L;
    case 'coldstart'
        tb = [0    120   180   900   960   1800];
        vb = [0    0     60    60    80    80  ];
        Nb = [600  600   1200  1200  1300  1300];
        Lb = [0.03 0.03  0.40  0.40  0.45  0.45];
        Ta = 5;   x0 = [5; 5; 5];    T_end = 1800;
        Lfun = @(t, L) L;
    otherwise
        error('drive_cycle: unknown scenario "%s"', name);
end

cyc.name  = lower(name);
cyc.T_end = T_end;
cyc.x0    = x0;
cyc.Ta    = Ta;
cyc.d     = @(t) local_d(t, tb, vb, Nb, Lb, Lfun, Ta, p.Tq_max);
end

function d = local_d(t, tb, vb, Nb, Lb, Lfun, Ta, Tq_max)
t  = min(max(t(:).', tb(1)), tb(end));
v  = interp1(tb, vb, t);
N  = interp1(tb, Nb, t);
L  = min(max(Lfun(t, interp1(tb, Lb, t)), 0), 1);
d  = [N; L*Tq_max; v; Ta*ones(size(t))];
end
