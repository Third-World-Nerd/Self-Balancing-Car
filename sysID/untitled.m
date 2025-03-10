% MATLAB Code to Read and Plot CSV Data
clc

% Read the CSV file
data = readtable('data1.csv');

% Extract columns
motor_input = data{:, 1}; % First column: Motor input
robot_angle = data{:, 2}; % Second column: Robot angle

motor_input = motor_input(1:123);
robot_angle = robot_angle(1:123);
robot_angle = robot_angle - robot_angle(1); % Add the first value to all elements


% Plot the data
figure;
subplot(2, 1, 1);
plot(motor_input, 'b', 'LineWidth', 1.5);
title('Motor Input');
xlabel('Time Index');
ylabel('Motor Input');
grid on;

subplot(2, 1, 2);
plot(robot_angle, 'r', 'LineWidth', 1.5);
title('Robot Angle');
xlabel('Time Index');
ylabel('Angle (degrees)');
grid on;

% Display a combined plot (optional)
figure;
plot(motor_input, 'b', 'LineWidth', 1.5);
hold on;
plot(robot_angle, 'r', 'LineWidth', 1.5);
title('Motor Input and Robot Angle');
xlabel('Time Index');
ylabel('Values');
legend('Motor Input', 'Robot Angle');
grid on;
hold off;
