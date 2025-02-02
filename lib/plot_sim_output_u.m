function plot_sim_output_u(logsout)
% Plot the sim output to a subplot figure.
% INPUT:
% out : sim output
figure
hold on
plot(logsout.Values.Time,rad2deg(squeeze(logsout.Values.data(1,1,:))), 'DisplayName','Stabilators');
plot(logsout.Values.Time,rad2deg(squeeze(logsout.Values.data(2,1,:))), 'DisplayName','Rudders');
plot(logsout.Values.Time,rad2deg(squeeze(logsout.Values.data(3,1,:))), 'DisplayName','Ailerons');
grid on;
xlabel('Time [s]');
ylabel('Magnitude [deg]');
legend show
end

