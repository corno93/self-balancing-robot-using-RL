
%% Code usign Ooi_example_editted.m BUT WITH VALUES FROM GAZEBO MODEL
% editted for use in simulink

clear all;
close all;
clc;
%%%%%%%%%%%%%%%%%%%%%%%%%%
% LQR_controlMD.m
% Simulation of Balancing Robot with LQR control
% This model includes the motor dynamics
% Author: Rich Chi Ooi (0248566)
% 12.08.2003
%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%
% Variable initialization
%%%%%%%%%%%%%%
 g=9.81; %Gravity(m/s^2)
 r=0.19; %Radius of wheel(m)
 Mw=2.6; %Mass of wheel(kg)
 Mp= 5.6; %Mass of body(kg)
 Iw=0.04693; %Inertia of the wheel(kg*m^2)
 Ip=0.0504; %Inertia of the body(kg*m^2)
 l=0.95/2; %Length to the body's centre of mass(m)

 %Motor's variables
 Km =  0.006123; %Motor torque constant (Nm/A)
 Ke = 0.006087; %Back EMF constant (Vs/rad)
 R = 3; %Nominal Terminal Resistance (Ohm)

% Va = voltage applied to motors for controlling the pendulum

%%%%%%%%%%%%%
% System Matrices
%%%%%%%%%%%%%
%pre-calculated to simplyfy the matrix
%Denominator for the A and B matrices
beta = (2*Mw+(2*Iw/r^2)+Mp);
alpha = (Ip*beta + 2*Mp*l^2*(Mw + Iw/r^2));
gamma = beta - (Mp^2 * (l^2))/(Ip + Mp*l^2);
zeta = ((Mp^2)*(l^2))/(beta) - Ip - Mp*l^2;
a23 = ((-Mp^2 * g * l^2)/(Ip + Mp*l^2))/gamma;
a25 = (Ip + Mp*l^2 - r*Mp*l)/gamma;
a26 = (Ip + Mp*l^2 - r*Mp*l)/gamma;
a43 = (-Mp*g*l)/zeta;
a45 = (2*r*Mw + 2*Iw + Mp*r - Mp*l)/zeta;
a46 = (2*r*Mw + 2*Iw + Mp*r - Mp*l)/zeta;
a52 = (-Km*Ke)/(R*r);
a62 = (-Km*Ke)/(R*r);


A = [0 1 0 0 0 0;
    0 0 a23 0 a25 a26;
    0 0 0 1 0 0;
    0 0 a43 0 a45 a46;
    0 a52 0 0 0 0;
    0 a62 0 0 0 0];
eigen_values = eig(A)

% B = [0 0;
%     0 0;
%     0 0;
%     0 0;
%     Km/r 0;
%     0 Km/r];
B = [0;
    0;
    0;
    0;
    Km/r;
    Km/r];

C = eye(6);

D = [0;
    0;
    0;
    0;
    0;
    0;];

Q = eye(6);
R = eye(1);
  
[K,p, e] = lqr(A,B,Q,R);
  
  
  %% Open loop next step prediction

  x_current = [0.0;                 % x
               0.0;                 % x_d
               -3.3189;           % phi
               -19.875;                 % phi_d
               --80;                 % torque m1 (-)
               +-80];                % torque m2 (+)
  dt = 0.05;
  
  u = 1;
  x_next = x_current + (A*x_current + B*u)*dt

