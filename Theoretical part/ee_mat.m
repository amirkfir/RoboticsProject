function T04 = ee_mat(theta)
% theta = [theta1, theta2, theta3, theta4]
%T04 is the homogeneus matrix at the end effector level

d1 = 0.05; % in meters
a1 = 0; 
a2 = 0.093;
a3 = 0.093;
a4 = 0.05;
alpha1 = 3.14/2; % Example link twist (in radians)
alpha2 = 0;
alpha3 = 0;
alpha4 = 0;
A = @(theta, d, a, alpha) [cos(theta), -sin(theta)*round(cos(alpha)), sin(theta)*round(sin(alpha)), a*cos(theta);
                           sin(theta), cos(theta)*round(cos(alpha)), -cos(theta)*round(sin(alpha)), a*sin(theta);
                           0, round(sin(alpha)), round(cos(alpha)), d;
                           0, 0, 0, 1];

% Transformation matrices for each joint
A1 = A(theta(1), d1, a1, alpha1);
A2 = A(theta(2), 0, a2, alpha2);
A3 = A(theta(3), 0, a3, alpha3);
A4 = A(theta(4), 0, a4, alpha4);

T04=A1*A2*A3*A4;