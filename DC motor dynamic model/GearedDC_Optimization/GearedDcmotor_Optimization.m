clc;
close all;
clear all;
% Experimental Data (Input Voltage and Measured RPM)
exp_voltage = [11.10, 11.00, 10.90, 10.80, 10.70, 10.60, 10.50, 10.40, 10.30, 10.20, 10.10, 10.00, 9.90, 9.80, 9.70, 9.60, 9.50, 9.40, 9.30, 9.20, 9.10, 9.00, 8.90, 8.80, 8.70, 8.60, 8.50, 8.40, 8.30, 8.20, 8.10, 8.00, 7.90, 7.80, 7.70, 7.60, 7.50, 7.40, 7.30, 7.20, 7.10, 7.00, 6.90, 6.80, 6.70, 6.60, 6.50, 6.40, 6.30, 6.20, 6.10, 6.00, 5.90, 5.80, 5.70, 5.60, 5.50, 5.40, 5.30, 5.20, 5.10, 5.00, 4.90, 4.80, 4.70, 4.60, 4.50, 4.40, 4.30, 4.20, 4.10, 4.00, 3.90, 3.80, 3.70, 3.60, 3.50, 3.40, 3.30, 3.20, 3.10, 3.00, 2.90, 2.80, 2.70, 2.60, 2.50, 2.40, 2.30, 2.20, 2.10, 2.00, 1.90, 1.80, 1.70, 1.60, 1.50, 1.40, 1.30, 1.20, 1.10, 1.00, 0.90]';
exp_rpm = [39.10, 39.02, 38.97, 38.91, 38.85, 38.86, 38.84, 38.79, 38.75, 38.72, 38.70, 38.70, 38.65, 38.59, 38.52, 38.46, 38.41, 38.36, 38.31, 38.29, 38.25, 38.20, 38.13, 38.11, 38.03, 37.98, 38.05, 37.97, 37.88, 37.91, 37.83, 37.74, 37.70, 37.67, 37.51, 37.29, 37.27, 37.24, 37.25, 37.24, 36.94, 36.87, 36.83, 36.73, 36.63, 36.55, 36.46, 36.31, 36.20, 36.08, 35.99, 35.86, 35.77, 35.67, 35.49, 35.36, 35.25, 35.14, 34.94, 34.78, 34.65, 34.46, 34.35, 34.23, 34.06, 33.96, 33.81, 33.60, 33.40, 33.19, 32.98, 32.69, 32.52, 32.33, 31.89, 31.61, 31.37, 31.06, 30.73, 30.37, 30.01, 29.55, 29.19, 28.80, 28.35, 27.71, 27.15, 26.63, 25.79, 24.92, 24.44, 23.39, 22.57, 21.72, 20.79, 19.25, 18.06, 16.80, 14.74, 13.10, 11.38, 8.37, 5.48]*0.1047198;
exp_voltage = flip(exp_voltage);
exp_rpm=flip(exp_rpm);
t_obj = 0:0.1:11;

% Interpolate exp_voltage and exp_rpm to match dimensions of t
exp_voltage_obj = interp1(linspace(0, 11, length(exp_voltage)), exp_voltage, t_obj);
exp_rpm_obj = interp1(linspace(0,11, length(exp_rpm)), exp_rpm, t_obj);

%t = 0:1:102;
% define the regularization parameter
lambda = 0.6;
% Define Transfer Function
s = tf('s');

% x[7]=[ J, B, L, Km, Kb, R, N]


% P_motor = (Km/N)/((J*s + B)*(L*s + R) + (Km*Kb)/(N^2));
% Define Objective Function
objfunc = @(x) sum((exp_rpm_obj' - lsim((x(4)/x(7))/((x(1)*s + x(2))*(x(3)*s + x(6)) + (x(4)*x(5))/(x(7)^2)), exp_voltage_obj, t_obj)).^2) + lambda*sum(x.^2);

% Parameter Bounds
lb = [0.01, 0.01, 0.1, 0.2, 0.05, 1.5,95]; % Lower bounds
ub = [inf, inf, inf, inf, inf, inf, 102]; % Upper bounds

% Initial Guess
x0 = [0.1, 0.103076, 0.100039, 0.401791, 0.176832, 2.342224, 99.78];
% Optimization Options
options =optimoptions('fmincon', 'Algorithm', 'sqp', 'StepTolerance', 1e-14, 'Display', 'iter', 'MaxIterations', 100);
% Perform Optimization
[x_opt, fval, EXITFLAG, output] = fmincon(objfunc, x0, [], [], [], [], lb, ub, [], options);
% Print Results
fprintf('Optimized Parameters:\n')
fprintf('J = %f kg*m^2\n', x_opt(1))
fprintf('B = %f N*m*s/rad\n', x_opt(2))
fprintf('J = %f H\n', x_opt(3))
fprintf('Km = %f Nm\n', x_opt(4))
fprintf('Kb = %f emf\n', x_opt(5))
fprintf('R = %f ohms\n', x_opt(6))
fprintf('N = %f ratio\n', x_opt(7))

% Simulate Model with Optimized Parameters
P_motor_opt = (x_opt(4)/x_opt(7))/((x_opt(1)*s + x_opt(2))*(x_opt(3)*s + x_opt(6)) + (x_opt(4)*x_opt(5))/(x_opt(7)^2));
[y_opt, t] = lsim(P_motor_opt,exp_voltage_obj, t_obj,x0);
speed_opt = y_opt*(60/(2*pi)); % rad/s to rpm

%Plot Experimental Data and Optimized Results
subplot(3,1,1)
plot(exp_voltage_obj, exp_rpm_obj/0.1047198, 'LineWidth', 2)
hold on
plot(exp_voltage_obj, speed_opt, 'LineWidth', 2)
xlabel('Input Voltage (V)')
ylabel('Speed (rpm)')
legend('Experimental Data', 'Optimized Model')
title('Geared DC Motor Step Response')

subplot(3,1,2)

speed = y_opt*(60/(2*pi)); % rad/s to rpm
subplot(3,1,2)
plot(t,speed_opt,'LineWidth', 2)
xlabel('Time (s)')
ylabel('Speed (rpm)')
title('Geared DC Motor Step Response')


% Plot the error
subplot(3,1,3)
% Calculate Error
error = (exp_rpm_obj- y_opt(:,1)')/0.1047198;
plot(exp_voltage_obj, error, 'LineWidth', 2)
xlabel('Input Voltage (V)')
ylabel('Error (rpm)')
title('Geared DC Motor Error Plot')

% theta = cumtrapz(t,y_opt) *(190/pi);
% subplot(3,1,3)
% plot(t,theta,'LineWidth', 2)
% xlabel('Time (s)')
% ylabel('Position (rad)')
% title('Geared DC Motor Position')

