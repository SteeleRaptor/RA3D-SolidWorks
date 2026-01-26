clc;clear;close all;
%Arm Lengths
%Calculate random angles for each joint angle
%Forward Kinematics for xyz
%3D Scatter plot


% The purpose of this code is determine the workspace of the arm. To do
% this random anlges will be selected from the angle range of each joint.
% The position at these random angles will then be plotted on a 3D graph.
% This will give a visualization of the range of the robotic arm.

%% Set the Joint angles ranges
for i = 1:1000
J1 = deg2rad(randi([-170, 170])); %Range is -170 to 170 degrees
J2 = deg2rad(randi([-42, 90]));  %Range is -42 to 90 degrees
J3 = deg2rad(randi([-89,52])); %Range is -89 to 52 degrees
J4 = deg2rad(randi([-165,165])); %Range is -165 to 165 degrees
J5 = deg2rad(randi([-105,105])); %Range is -105 to 105 degrees
J6 = deg2rad(randi([-155,155])); %Range is -155 to 155 degrees
% D-H parameters Table
theta = [J1 (J2 - (pi/2)) (J3 + pi) J4 J5 J6];%Theta for MK3
alpha = [(-pi/2) 0 (pi/2) (-pi/2) (pi/2) 0];%Alpha for MK3
d = [169.77 0 0 222.63 0 41];%D distance of MK3 
a = [64.2 305 0 0 0 0];%a distance of MK3
%Calculate the 
T1 = Transformation(theta(1), d(1), a(1), alpha(1));% Base to joint 1
T2 = Transformation(theta(2), d(2), a(2), alpha(2));% joint 1 to joint 2
T3 = Transformation(theta(3), d(3), a(3), alpha(3));%joint 2 to joint 3
T4 = Transformation(theta(4), d(4), a(4), alpha(4));%joint 3 to joint 4
T5 = Transformation(theta(5), d(5), a(5), alpha(5));%joint 4 to joint 5
T6 = Transformation(theta(6), d(6), a(6), alpha(6));%joint 5 to joint 6
Tfinal = (((((T1*T2)*T3)*T4)*T5)*T6);% Final transfermation matrix.
figure(1)
plot3(Tfinal(1,4),Tfinal(2,4),Tfinal(3,4),"o");
hold on
figure(2)
plot(Tfinal(1,4),Tfinal(2,4),"o") %x-axis vs y-axis
xlabel("X-axis")
ylabel("y-label")
hold on
figure(3)
plot(Tfinal(1,4),Tfinal(3,4),"o")% x-axis vs z-axis
xlabel("x-label")
ylabel("y-label")
hold on
figure(4)
plot(Tfinal(2,4),Tfinal(3,4),"o")%Y-axis vs z-axis
xlabel("y-axis")
ylabel("z-axis")
hold on
end
%% Functions
% Transformation function
function [T] = Transformation(theta, d, a, alpha)
    T = [cos(theta) -sin(theta)*cos(alpha) sin(theta)*sin(alpha) a*cos(theta);
         sin(theta) cos(theta)*cos(alpha) -cos(theta)*sin(alpha) a * sin(theta);
         0 sin(alpha) cos(alpha) d;
         0 0 0 1];
end
