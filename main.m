%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                         %
%                       APPM 3310                         %
%                                                         %
%                     Final Project                       %
%       Kalman Filter, Riemann Summs, and Kinematics      %
%                                                         %
%                      11/27/2022                         %
%                                                         %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Set Environment
clear
clc

%% Set the variables

% Import the Acceleration Data
[~, x_accel, ~, ~] = load_accel_data("Data/ADXL357_DATA_2.csv");

% Convert the acceleration data to [m/s^2]
x_accel = x_accel .* 9.81;

% Set the Time for the imported data
t = linspace(0,10,length(x_accel));

%% Kalman Filter

% Get the standard deviation of the acceleration data
std_accel = std(x_accel);

% Set P_0_0
P_0_0 = zeros(3,3);

% Call the Kalman Filter
kf_state = KF_accel(t, x_accel, std_accel, P_0_0);

%% Kinematic Models

% Preallocate vectors and set initial positions
n = length(x_accel);
V_k = zeros(1,n);
V_k(1) = 0;
X_k = zeros(1,n);
X_k(1) = 0;

% Calculate velocities and positions using 1D kinematic equations
for i = 2:n
    dt = t(i) - t(i-1);
    V_k(i) = V_k(i-1) + x_accel(i)*dt;
    X_k(i) = X_k(i-1) + V_k(i)*dt + (1/2)*x_accel(i)*dt^2;
end
 
%% Cumulative Trapezoidal
V_r = cumtrapz(t, x_accel);
X_r = cumtrapz(t, V_r);

%% Plots

figure;
title("Acceleration, Velocity, Position of Data")
tiled = tiledlayout(3,1);
xlab = xlabel(tiled,{'Time','$\mathbf{s}$'},'interpreter', 'latex');

nexttile
hold on
ylabel({'Acceleration','$\mathbf{m/s^{2}}$'},'interpreter', 'latex');
plot(t,x_accel)
% plot(t,kf_state(3,:))
legend('Truth')
hold off

nexttile
hold on
ylabel({'Velocity','$\mathbf{m/s}$'},'interpreter', 'latex');
plot(t,kf_state(2,:))
plot(t(1:length(V_r)), V_r)
plot(t, V_k)
legend('KF','Trapezoidal', 'Kinematic','Location','eastoutside')
hold off

nexttile
hold on
ylabel({'Position','$\mathbf{m}$'},'interpreter', 'latex');
plot(t, kf_state(1,:))
plot(t(1:length(X_r)), X_r)
plot(t, X_k)
legend('KF','Trapezoidal', 'Kinematic','Location','eastoutside')
hold off


%% Functions