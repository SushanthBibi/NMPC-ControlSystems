function [x_next, y] = rk4_step(x, u, d, dt, p)
%RK4_STEP  One 4th-order Runge-Kutta step of thermal_model with u, d held.
[k1, y] = thermal_model(x,             u, d, p);
k2      = thermal_model(x + dt/2*k1,   u, d, p);
k3      = thermal_model(x + dt/2*k2,   u, d, p);
k4      = thermal_model(x + dt*k3,     u, d, p);
x_next  = x + dt/6*(k1 + 2*k2 + 2*k3 + k4);
end
