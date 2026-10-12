function mdl = build_thermal_simulink(ctrl_name, scenario)
%BUILD_THERMAL_SIMULINK  Build the closed-loop cooling-system Simulink model.
%
%   mdl = build_thermal_simulink(ctrl_name, scenario)
%     ctrl_name  'nmpc' | 'pid' | 'conventional'   (default 'nmpc')
%     scenario   drive_cycle name                   (default 'hill')
%
%   Creates (and saves) thermal_<ctrl_name>.slx using only basic Simulink
%   library blocks:
%
%     Clock -> [Disturbance]  d = [N; Tq; v; T_amb]
%     [x; u; d] -> [Plant dx = f(x,u,d)] -> Integrator -> x
%     controller:
%        'pid'          T_c - T_ref -> Discrete PID Controller (clamping
%                       anti-windup, output 0..3) -> split range -> u
%        'nmpc'         [x; d; t] -> NMPC (Interpreted MATLAB Fcn, Ts=5 s) -> u
%        'conventional' [x; d; t] -> thermostat / fan clutch logic -> u
%
%   The plant and NMPC blocks call the same .m files as the MATLAB
%   scripts, so Simulink and MATLAB results match.
if nargin < 1, ctrl_name = 'nmpc'; end
if nargin < 2, scenario  = 'hill'; end
root = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(root, 'model'), fullfile(root, 'controllers'), ...
        fullfile(root, 'sim'), fullfile(root, 'simulink'));

p   = thermal_params();
cyc = drive_cycle(scenario);
assignin('base', 'SL_SCENARIO',   scenario);
assignin('base', 'SL_CONTROLLER', lower(ctrl_name));

mdl = ['thermal_' lower(ctrl_name)];
if bdIsLoaded(mdl), close_system(mdl, 0); end
slx = fullfile(root, 'simulink', [mdl '.slx']);
if exist(slx, 'file'), delete(slx); end
new_system(mdl); open_system(mdl);
set_param(mdl, 'Solver', 'ode4', 'FixedStep', num2str(p.dt), ...
    'StopTime', num2str(cyc.T_end), 'ReturnWorkspaceOutputs', 'off');

IMF = 'simulink/User-Defined Functions/Interpreted MATLAB Function';

% ---- sources ----
add_block('simulink/Sources/Clock', [mdl '/Clock'], 'Position', [30 40 60 70]);
add_block(IMF, [mdl '/Disturbance'], 'MATLABFcn', 'sl_disturbance(u)', ...
    'OutputDimensions', '4', 'Position', [110 35 230 75]);
add_line(mdl, 'Clock/1', 'Disturbance/1');

% ---- plant ----
add_block('simulink/Signal Routing/Mux', [mdl '/MuxPlant'], 'Inputs', '3', ...
    'Position', [520 180 525 260]);
add_block(IMF, [mdl '/Plant f(x,u,d)'], 'MATLABFcn', 'sl_plant(u)', ...
    'OutputDimensions', '3', 'Position', [570 200 690 240]);
add_block('simulink/Continuous/Integrator', [mdl '/Integrator'], ...
    'InitialCondition', mat2str(cyc.x0), 'Position', [730 205 760 235]);
add_line(mdl, 'MuxPlant/1', 'Plant f(x,u,d)/1');
add_line(mdl, 'Plant f(x,u,d)/1', 'Integrator/1');
add_line(mdl, 'Integrator/1', 'MuxPlant/1', 'autorouting', 'on');
add_line(mdl, 'Disturbance/1', 'MuxPlant/3', 'autorouting', 'on');

% ---- controller ----
switch lower(ctrl_name)
    case 'pid'
        g = p.pid;
        add_block('simulink/Signal Routing/Demux', [mdl '/DemuxX'], 'Outputs', '3', ...
            'Position', [820 300 825 360]);
        add_block('simulink/Sources/Constant', [mdl '/T_ref'], 'Value', num2str(p.T_ref), ...
            'Position', [820 390 850 410]);
        add_block('simulink/Math Operations/Sum', [mdl '/Error'], 'Inputs', '+-', ...
            'Position', [880 320 900 340]);
        add_block('simulink/Discrete/Discrete PID Controller', [mdl '/PID'], ...
            'P', num2str(g.Kp), 'I', num2str(g.Ki), 'D', num2str(g.Kd), ...
            'N', num2str(1/g.Tf), 'SampleTime', num2str(g.Ts), ...
            'LimitOutput', 'on', 'UpperSaturationLimit', '3', ...
            'LowerSaturationLimit', '0', 'AntiWindupMode', 'clamping', ...
            'Position', [930 310 990 350]);
        add_block(IMF, [mdl '/Split range'], 'MATLABFcn', 'sl_split_range(u)', ...
            'OutputDimensions', '3', 'Position', [1020 310 1120 350]);
        add_block('simulink/Sinks/Terminator', [mdl '/T2'], 'Position', [860 340 870 350]);
        add_block('simulink/Sinks/Terminator', [mdl '/T3'], 'Position', [860 360 870 370]);
        add_line(mdl, 'Integrator/1', 'DemuxX/1', 'autorouting', 'on');
        add_line(mdl, 'DemuxX/1', 'Error/1');
        add_line(mdl, 'DemuxX/2', 'T2/1');
        add_line(mdl, 'DemuxX/3', 'T3/1');
        add_line(mdl, 'T_ref/1', 'Error/2', 'autorouting', 'on');
        add_line(mdl, 'Error/1', 'PID/1');
        add_line(mdl, 'PID/1', 'Split range/1');
        add_line(mdl, 'Split range/1', 'MuxPlant/2', 'autorouting', 'on');
        u_src = 'Split range/1';
    otherwise
        Ts = p.dt;
        if strcmpi(ctrl_name, 'nmpc'), Ts = p.mpc.Ts; end
        add_block('simulink/Signal Routing/Mux', [mdl '/MuxCtrl'], 'Inputs', '3', ...
            'Position', [300 60 305 140]);
        add_block(IMF, [mdl '/Controller'], 'MATLABFcn', 'sl_controller(u)', ...
            'OutputDimensions', '3', 'SampleTime', num2str(Ts), ...
            'Position', [350 80 470 120]);
        add_line(mdl, 'Integrator/1', 'MuxCtrl/1', 'autorouting', 'on');
        add_line(mdl, 'Disturbance/1', 'MuxCtrl/2', 'autorouting', 'on');
        add_line(mdl, 'Clock/1', 'MuxCtrl/3', 'autorouting', 'on');
        add_line(mdl, 'MuxCtrl/1', 'Controller/1');
        add_line(mdl, 'Controller/1', 'MuxPlant/2', 'autorouting', 'on');
        u_src = 'Controller/1';
end

% ---- logging ----
add_block('simulink/Sinks/To Workspace', [mdl '/log_x'], 'VariableName', 'sl_x', ...
    'SaveFormat', 'Array', 'Position', [820 200 880 220]);
add_block('simulink/Sinks/To Workspace', [mdl '/log_u'], 'VariableName', 'sl_u', ...
    'SaveFormat', 'Array', 'Position', [820 440 880 460]);
add_block('simulink/Sinks/To Workspace', [mdl '/log_d'], 'VariableName', 'sl_d', ...
    'SaveFormat', 'Array', 'Position', [300 0 360 20]);
add_block('simulink/Sinks/To Workspace', [mdl '/log_t'], 'VariableName', 'sl_t', ...
    'SaveFormat', 'Array', 'Position', [110 0 170 20]);
add_block('simulink/Sinks/Scope', [mdl '/Temperatures'], 'Position', [820 240 850 270]);
add_line(mdl, 'Integrator/1', 'log_x/1', 'autorouting', 'on');
add_line(mdl, 'Integrator/1', 'Temperatures/1', 'autorouting', 'on');
add_line(mdl, u_src, 'log_u/1', 'autorouting', 'on');
add_line(mdl, 'Disturbance/1', 'log_d/1', 'autorouting', 'on');
add_line(mdl, 'Clock/1', 'log_t/1', 'autorouting', 'on');

save_system(mdl, slx);
fprintf('Built and saved %s.slx (scenario "%s")\n', mdl, scenario);
end
