% Design a linear controlelr the trim point (operating point)

clear
close all
clc

% Get all configurational and initial parameters
run config.m;

%load the linear model (designed in LinearizeModel.m)
%linsys is the 9 state model
%linsys6 is the 6 state model
load TrimPoint_Data/LM4.mat; 

%% Plot all Poles of the system
figure
hold on
grid on
poles = pole(linsys6);
labels = {'1-Short Period', '2-Short Period', '3-Dutch Roll', '4-Dutch Roll', '5-Roll Subsidence','6-Spiral'}; % Labels for each point
for i = 1:length(poles)
    text(real(poles(i)) + 0.01, imag(poles(i)), labels{i}, 'FontSize', 10); % Slight offset for better visibility
end
plot(poles,'+');

%% Plot the compass plot for the 4 eigenmotions
figure
[V, D] = eig(linsys6.A);
counter = 1;
for i = [1,3,5,6]
    subplot(2,2,counter);
    c = compass(V(:,i));
    % Farbe und Dicke der Pfeile festlegen:
    c(1).Color = 'r'; c(1).LineWidth = 2;
    c(2).Color = 'g'; c(2).LineWidth = 2;
    c(3).Color = 'b'; c(3).LineWidth = 2;
    c(4).Color = 'c'; c(4).LineWidth = 2;
    c(5).Color = 'm'; c(4).LineWidth = 2;
    c(6).Color = 'k'; c(4).LineWidth = 2;
    legend(xstates6,"Location","west");
    counter = counter +1;
end

   
