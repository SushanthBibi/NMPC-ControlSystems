function d = sl_disturbance(t)
%SL_DISTURBANCE  Simulink wrapper: drive-cycle disturbance d(t) = [N;Tq;v;T_amb].
persistent cyc name
sc = evalin('base', 'SL_SCENARIO');
if isempty(cyc) || ~strcmp(name, sc), cyc = drive_cycle(sc); name = sc; end
d = cyc.d(t);
end
