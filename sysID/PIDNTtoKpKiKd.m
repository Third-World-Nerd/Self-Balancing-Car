% Given PID parameters
P = 1.6092091535904; % Proportional gain
I = 11.715656092041; % Integral gain
D = -0.486044961313621; % Derivative gain
N = 3.1402122120234; % Derivative filter coefficient
T = 0.01; % Sampling time

% Simplified PID gains
Kp = P + D * N;
Ki = T * (P * N + I) - 2 * (P + D * N);
Kd = (1 - N * T) * (P - I * T) + D * N;

% Display results
disp(['Kp: ', num2str(Kp)]);
disp(['Ki: ', num2str(Ki)]);
disp(['Kd: ', num2str(Kd)]);
