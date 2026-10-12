function u = sl_controller(in)
%SL_CONTROLLER  Simulink wrapper: in = [x(3); d(4); t] -> u = [fan; pump; valve].
%   Runs the NMPC or the conventional controller selected by the base
%   workspace variable SL_CONTROLLER. Memory is reset when t == 0.
persistent p mem cyc
x = in(1:3); d = in(4:7); t = in(8);
if isempty(p) || t == 0
    p = thermal_params(); mem = [];
    cyc = drive_cycle(evalin('base', 'SL_SCENARIO'));
end
switch evalin('base', 'SL_CONTROLLER')
    case 'nmpc',         [u, mem] = nmpc_controller(x, t, mem, p, cyc.d);
    case 'conventional', [u, mem] = ctrl_conventional(x, d, mem, p);
    otherwise, error('sl_controller: unsupported controller');
end
end
