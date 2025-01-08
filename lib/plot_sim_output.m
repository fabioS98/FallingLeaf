function plot_sim_output(out)
% Plot the sim output to a subplot figure.
% INPUT:
% out : sim output
figure
hold on
subplot(2,1,1)
plot(rad2deg(squeeze(out.state.data(2,1,:))),rad2deg(squeeze(out.state.data(3,1,:))), 'DisplayName','Trajectory');
grid on;
xlabel('sideslip angle [deg]');
ylabel('angle of attack [deg]');
legend show

subplot(2,1,2)
plot(out.tout, rad2deg(squeeze(out.state.data(4,1,:))), 'r-', 'DisplayName','Roll Rate');
hold on
plot(out.tout, rad2deg(squeeze(out.state.data(6,1,:))), 'b--', 'DisplayName', 'Yaw Rate');
grid on;
xlabel('Time [sec]');
ylabel('Rate [deg/sec]');
legend show;
end

