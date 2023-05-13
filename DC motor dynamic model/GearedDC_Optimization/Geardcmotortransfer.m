clear all;
close all;
clc;
% Geared DC Motor Parameters
R = 1.5;%2.1429; % armature resistance
L = 0.000000;%0.5; % armature inductance
Km = 0.2; %0.3997; % motor torque constant
Kb = 0.05;%0.12; % back EMF constant
J = 0.027678; %8.125e-6  % rotor inertia
B = 0.002513 ;%0.03997; % damping coefficient
N = 95.000000;%99.78; % gear ratio

% Transfer Function
s = tf('s');
P_motor = (Km/N)/((J*s + B)*(L*s + R) + (Km*Kb)/(N^2));

% Input Voltage
t = 0:0.01:40 ;

disp(t);
%u = 11.10*(mod(t,20))*ones(size(t)); % input voltage 
t = 0:0.01:100;
u = zeros(size(t));
u(mod(t,100) < 60) = 11.10;
u(mod(t,100) >= 60) = 6.0;


subplot(3,1,1)
plot(t,u)
xlabel('Time (s)')
ylabel('Voltage (V)')
title('Input Voltage')

% Step Response (Speed)
[y,t] = lsim(P_motor,u,t);
speed = y*(60/(2*pi)); % rad/s to rpm
subplot(3,1,2)
plot(t,speed)
xlabel('Time (s)')
ylabel('Speed (rpm)')
title('Geared DC Motor Step Response')

% Position
theta = cumtrapz(t,y) *(190/pi);
subplot(3,1,3)
plot(t,theta)
xlabel('Time (s)')
ylabel('Position (rad)')
title('Geared DC Motor Position')
