%RUN_SIMULINK_DEMO  Build and run the Simulink closed loop for each controller.
clear; clc; close all;
here = fileparts(mfilename('fullpath'));
addpath(here);
scenario = 'hill';
ctrls    = {'conventional', 'pid', 'nmpc'};
p   = []; res = struct();
for j = 1:numel(ctrls)
    mdl = build_thermal_simulink(ctrls{j}, scenario);
    if isempty(p), p = thermal_params(); end
    clear sl_controller sl_disturbance       % reset persistent memory
    sim(mdl);
    res.(ctrls{j}) = struct('t', sl_t, 'x', sl_x, 'u', sl_u);
end

figure('Name', ['Simulink: ' scenario]);
subplot(2,1,1); hold on;
for j = 1:numel(ctrls), plot(res.(ctrls{j}).t, res.(ctrls{j}).x(:,1)); end
plot(xlim, p.T_ref*[1 1], 'k--'); ylabel('T_c [degC]'); grid on; legend(ctrls{:});
subplot(2,1,2); hold on;
for j = 1:numel(ctrls), plot(res.(ctrls{j}).t, res.(ctrls{j}).u(:,1)); end
ylabel('fan [-]'); xlabel('time [s]'); grid on;
