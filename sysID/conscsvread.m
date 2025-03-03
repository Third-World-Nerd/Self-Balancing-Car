% MATLAB Code to Read and Plot CSV Data
clc

% Read the CSV file
data = readtable('consinput.csv');

% Extract columns
motor_inputcons = data{:, 1}; % First column: Motor input
robot_anglecons = data{:, 2}; % Second column: Robot angle

motor_inputcons = motor_inputcons(1:34);
robot_anglecons = robot_anglecons(1:34);
robot_anglecons = robot_anglecons - robot_anglecons(1); % Add the first value to all elements


% Plot the data
figure;
subplot(2, 1, 1);
plot(motor_inputcons, 'b', 'LineWidth', 1.5);
title('Motor Input');
xlabel('Time Index');
ylabel('Motor Input');
grid on;

subplot(2, 1, 2);
plot(robot_anglecons, 'r', 'LineWidth', 1.5);
title('Robot Angle');
xlabel('Time Index');
ylabel('Angle (degrees)');
grid on;

% Display a combined plot (optional)
figure;
plot(motor_inputcons, 'b', 'LineWidth', 1.5);
hold on;
plot(robot_anglecons, 'r', 'LineWidth', 1.5);
title('Motor Input and Robot Angle');
xlabel('Time Index');
ylabel('Values');
legend('Motor Input', 'Robot Angle');
grid on;
hold off;
