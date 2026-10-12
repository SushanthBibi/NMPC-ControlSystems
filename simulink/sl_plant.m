function dx = sl_plant(in)
%SL_PLANT  Simulink wrapper: in = [x(3); u(3); d(4)] -> dx = f(x,u,d).
persistent p
if isempty(p), p = thermal_params(); end
dx = thermal_model(in(1:3), in(4:6), in(7:10), p);
end
