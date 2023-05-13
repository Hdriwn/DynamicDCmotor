clear all;
close all;
clc;

% Motor parameters
R = 1.5;         % armature resistance (ohms)
L = 0.000000;    % armature inductance (H)
Km = 0.2;        % motor torque constant (Nm/A)
Kb = 0.05;       % back EMF constant (V/(rad/s))
J = 0.027678;    % rotor inertia (kg*m^2)
B = 0.002513;    % damping coefficient (Nm/(rad/s))

% Transfer function
s = tf('s');
P_motor = Km/((J*s + B)*(L*s + R) + Km*Kb);

% Input voltage
t = 0:0.01:10;
u = 11.10*ones(size(t)); % input voltage (V)

% Step response (position in degrees)
[y, t] = lsim(P_motor, u, t);
position = y*(180/pi);   % rad to deg

% Plotting input voltage and position
subplot(2,1,1)
plot(t,u)
xlabel('Time (s)')
ylabel('Voltage (V)')
title('Input Voltage')

subplot(2,1,2)
plot(t,position)
xlabel('Time (s)')
ylabel('Position (deg)')
title('Ungeared DC Motor Step Response')
