%RUN_COMPARISON  Main script: conventional vs PID vs NMPC thermal management.
%
%   Runs every controller on every scenario, prints the KPI table from the
%   project methodology and plots the results.
%
%   Tip: the NMPC is the slow part. With the Optimization Toolbox it uses
%   fmincon (fast); without it, fminsearch from base MATLAB (slower).
%   Shorten the list below while experimenting.
clear; clc; close all;
root = fileparts(mfilename('fullpath'));
addpath(fullfile(root, 'model'), fullfile(root, 'controllers'), fullfile(root, 'sim'));

p           = thermal_params();
scenarios   = {'highway', 'hill', 'urban', 'coldstart'};
controllers = {'conventional', 'pid', 'nmpc'};

R = cell(numel(scenarios), numel(controllers));
for i = 1:numel(scenarios)
    for j = 1:numel(controllers)
        fprintf('Simulating %-10s with %-12s ... ', scenarios{i}, controllers{j});
        R{i, j} = simulate_system(controllers{j}, scenarios{i}, p);
        fprintf('done (%.1f s)\n', R{i, j}.cpu_time);
    end
end

% ---------------- KPI table ----------------
for i = 1:numel(scenarios)
    fprintf('\n=== Scenario: %s ===\n', scenarios{i});
    fprintf('%-13s %8s %8s %8s %8s %8s %8s %8s %9s %8s\n', 'controller', ...
        'Tc std', 'Tc rmse', 'To std', 'Tc peak', 'To peak', 'E fan', ...
        'E pump', 'fuel', 'warm-up');
    fprintf('%-13s %8s %8s %8s %8s %8s %8s %8s %9s %8s\n', '', ...
        '[K]', '[K]', '[K]', '[degC]', '[degC]', '[kWh]', '[kWh]', '[kg]', '[s]');
    for j = 1:numel(controllers)
        k = R{i, j}.kpi;
        fprintf('%-13s %8.2f %8.2f %8.2f %8.1f %8.1f %8.3f %8.3f %9.3f %8.0f\n', ...
            controllers{j}, k.Tc_std, k.Tc_rmse, k.To_std, k.Tc_peak, k.To_peak, ...
            k.E_fan, k.E_pump, k.fuel, k.warmup_time);
    end
    k0 = R{i, 1}.kpi; kn = R{i, end}.kpi;
    fprintf('%s vs %s: cooling energy %+.1f %%, fuel %+.2f %%\n', ...
        controllers{end}, controllers{1}, 100*(kn.E_cool - k0.E_cool)/max(k0.E_cool, eps), 100*(kn.fuel - k0.fuel)/k0.fuel);
end

% ---------------- Plots ----------------
for i = 1:numel(scenarios)
    plot_results(R(i, :), p);
end
