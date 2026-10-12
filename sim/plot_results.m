function plot_results(runs, p)
%PLOT_RESULTS  Compare several closed-loop runs on the same scenario.
col = {'b', 'g', 'r', 'm', 'c'};
figure('Name', ['Scenario: ' runs{1}.scenario]);
names = cellfun(@(r) r.name, runs, 'UniformOutput', false);

subplot(3,2,1); hold on;
for j = 1:numel(runs), plot(runs{j}.t, runs{j}.x(1,:), col{j}); end
plot(runs{1}.t([1 end]), p.T_ref*[1 1], 'k--');
plot(runs{1}.t([1 end]), p.T_c_max*[1 1], 'k:');
ylabel('T_c [degC]'); title('Coolant temperature'); grid on; legend(names{:}, 'Location', 'best');

subplot(3,2,2); hold on;
for j = 1:numel(runs), plot(runs{j}.t, runs{j}.x(3,:), col{j}); end
ylabel('T_o [degC]'); title('Oil temperature'); grid on;

subplot(3,2,3); hold on;
for j = 1:numel(runs), plot(runs{j}.t, runs{j}.u(1,:), col{j}); end
ylabel('u_f [-]'); title('Fan speed'); grid on; ylim([-0.05 1.05]);

subplot(3,2,4); hold on;
for j = 1:numel(runs), plot(runs{j}.t, runs{j}.u(2,:), col{j}); end
ylabel('u_p [-]'); title('Pump speed'); grid on; ylim([-0.05 1.05]);

subplot(3,2,5); hold on;
for j = 1:numel(runs), plot(runs{j}.t, runs{j}.u(3,:), col{j}); end
ylabel('x_v [-]'); title('Thermostat valve'); grid on; ylim([-0.05 1.05]); xlabel('time [s]');

subplot(3,2,6); hold on;
for j = 1:numel(runs)
    dt = runs{j}.t(2) - runs{j}.t(1);
    plot(runs{j}.t, cumsum(runs{j}.y(1,:) + runs{j}.y(2,:)) * dt / 3.6e6, col{j});
end
ylabel('[kWh]'); title('Cumulative fan + pump energy'); grid on; xlabel('time [s]');
end
