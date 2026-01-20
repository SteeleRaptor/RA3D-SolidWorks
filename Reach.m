clc;clear;close all;
%Arm Lengths
%Calculate random angles for each joint angle
%Forward Kinematics for xyz
%3D Scatter plot

%% Purpose Statement
% The purpose of this code is to calculate the forward kinematics of AR4
% MK3 robotic arm. This will use the Denavit Hartenberg Method to determine
% the transfer frunction of each joints. Since this is foward Kinematics
% the input will be the joint angles for each arm. The output will be
% position(x,y,z), yaw (rotation about z-axis), pitch (rotation about
% y-axis), and roll (rotation about the x-axis).

%% Set Joint angles
%These values are the inputs for the foward kinematics and include all
%joint angles.
% Should be in radians
J1 = 0; %Range is -170 to 170 degrees
J2 = 0; %Range is -42 to 90 degrees
J3 = 0; % Range is -89 to 52 degrees
J4 = 0; % Range is -165 to 165 degrees
J5 = 0;% Range is -105 to 105 degrees
J6 = 0;% Range is -155 to 155 degrees

%% D-H parameters Table
theta = [J1 (J2 - (pi/2)) (J3 + pi) J4 J5 J6];%Theta for MK3
alpha = [(-pi/2) 0 (pi/2) (-pi/2) (pi/2) 0];%Alpha for MK3
d = [169.77 0 0 222.63 0 41];%D distance of MK3 
a = [64.2 305 0 0 0 0];%a distance of MK3

%% Determine the transfermation functions
%This is used to calculate the transfermation matrix for each joint.
T1 = Transformation(theta(1), d(1), a(1), alpha(1));% Base to joint 1
T2 = Transformation(theta(2), d(2), a(2), alpha(2));% joint 1 to joint 2
T3 = Transformation(theta(3), d(3), a(3), alpha(3));%joint 2 to joint 3
T4 = Transformation(theta(4), d(4), a(4), alpha(4));%joint 3 to joint 4
T5 = Transformation(theta(5), d(5), a(5), alpha(5));%joint 4 to joint 5
T6 = Transformation(theta(6), d(6), a(6), alpha(6));%joint 5 to joint 6
Tfinal = T1*T2*T3*T4*T5*T6% Final transfermation matrix.
%% Calculate yaw pitch and roll
P = CalPitch(Tfinal);%y- axis
R = CalRoll(P,Tfinal);%x-axis
Y = CalYaw(P,Tfinal);%Theta
P = rad2deg(P)
R = rad2deg(R)
Y = rad2deg(Y)
%% Functions
% Transfermation function
function [T] = Transformation(theta, d, a, alpha)
    T = [cos(theta) -sin(theta)*cos(alpha) sin(theta)*cos(alpha) a*cos(theta);
         sin(theta) cos(theta)*cos(alpha) -cos(theta)*sin(alpha) a * sin(theta);
         0 sin(alpha) cos(alpha) d;
         0 0 0 1];
end
%Yaw, pitch, and roll function
%Calculates pitch(y)
function [P] = CalPitch(T)
    %P = atan2(-T(3,1),sqrt((T(1,1)^2)+(T(2,1)^2)));
    P = atan2(T(1,3),T(2,3));
end
%Calculates roll(x)
function [R] = CalRoll(P,T)
    R = atan2(T(3,2)/cos(P),T(3,3)/cos(P));
    %R = atan2(T(3,3)/cos(P),T(3,2)/cos(P));
    %R = atan2(-T(1,1)*sin(P)+T(2,1)*cos(P), (-T(1,2)*sin(P)+T(2,2)*cos(P)));
end
%calculates yaw(z)
function [Y] = CalYaw (P,T)
    Y = atan2((T(1,3)*cos(P)+T(2,3)*sin(P)),T(3,3));
    Y = atan2(T(2,1)/cos(P),T(1,1)/cos(P));
end
