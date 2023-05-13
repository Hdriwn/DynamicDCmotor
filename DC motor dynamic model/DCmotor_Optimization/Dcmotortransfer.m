% Clear workspace and close all figures
clear all; close all; clc;

% Define motor parameters
Rp = 10.5263 ; % Armature resistance (ohms)
Lp = 0.000001 ; % Armature inductance (H)
Kmp = 0.0020 ; % Motor torque constant (Nm/A)
Kbp = 0.0021; % Back EMF constant (V/(rad/s))
Jp = 0.000001; % Rotor inertia (kg*m^2)
Bp = 0.0000001; % Damping coefficient (Nm/(rad/s))

R = 10.411519 ; % Armature resistance (ohms)
L = 0.000100 ; % Armature inductance (H)
Km = 0.041728 ; % Motor torque constant (Nm/A)
Kb = 0.000465; % Back EMF constant (V/(rad/s))
J = 0.001669; % Rotor inertia (kg*m^2)
B = 0.000000; % Damping coefficient (Nm/(rad/s))

fprintf('Original Parameters:\n')
fprintf('Km = %f Nm\n', Kmp)
fprintf('R = %f ohms\n', Rp)
fprintf('Kb = %f emf\n', Kbp)
fprintf('J = %f kg*m^2\n', Jp)
fprintf('B = %f N*m*s/rad\n', Bp)
fprintf('L = %f H\n', Lp)

fprintf('Optimized Parameters:\n')
fprintf('Km = %f Nm\n', Km)
fprintf('R = %f ohms\n', R)
fprintf('Kb = %f emf\n', Kb)
fprintf('J = %f kg*m^2\n', J)
fprintf('B = %f N*m*s/rad\n', B)
fprintf('L = %f H\n', L)



% Calculate transfer function of motor
s = tf('s');
P_motor = Km/((J*s + B)*(L*s + R) + Km*Kb);
PP_motor = Kmp/((Jp*s + Bp)*(Lp*s + Rp) + Kmp*Kbp);

% Define input voltage
% t = 0:0.01:10;
t=linspace(0, 10, 13377) ;
u = 9.5*ones(size(t));

% Simulate step response of motor
[y, t] = lsim(P_motor, u, t);
position = y*(180/pi); % Convert from radians to degrees

[yp, tp] = lsim(PP_motor, u, t);
positionp = yp*(180/pi); % Convert from radians to degrees

% Plot input voltage and position of motor
subplot(2,1,1)
plot(t,u)
xlabel('Time (s)')
ylabel('Voltage (V)')
title('Input Voltage')

subplot(2,1,2)
plot(tp,positionp, 'LineWidth', 2)
hold on
plot(t,position,  'LineWidth', 2)
xlabel('Time (s)')
ylabel('Position (deg)')
legend('original','optimized')
title('Ungeared DC Motor Step Response') 