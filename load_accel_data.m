function [temp, accel_x, accel_y, accel_z] = load_accel_data(filename)
    % Load data to matrix
    raw = readmatrix(filename);

    % Average temperature of test
    temp = mean(raw(:,1)); % [deg C] 

    % Number of data points n
    n = length(raw);
    n_vec = 1:n;

    % Split data
    accel_x = raw(:,2);
    accel_y = raw(:,3);
    accel_z = raw(:,4);

    figure;
    subplot(3,1,1)
    plot(n_vec, accel_x);
    ylabel("X Acceleration [g]");

    subplot(3,1,2)
    plot(n_vec, accel_y);
    ylabel("Y Acceleration [g]");

    subplot(3,1,3)
    plot(n_vec, accel_z);
    ylabel("Z Acceleration [g]");
end