%% Part 1 Robotics project assignment
close all
clear all
clc

% DH Parameters for a 4 DOF robot arm
syms theta1 theta2 theta3 theta4; % Joint angles
d1 = 0.05; % in meters
a1 = 0; 
a2 = 0.093;
a3 = 0.093;
a4 = 0.05;
a5 = 0.035;
alpha1 = 3.14/2; % Example link twist (in radians)
alpha2 = 0;
alpha3 = 0;
alpha4 = 0;
d5_y = 0.045;

% Define the transformation matrix using DH parameters
A = @(theta, d, a, alpha) [cos(theta), -sin(theta)*round(cos(alpha)), sin(theta)*round(sin(alpha)), a*cos(theta);
                           sin(theta), cos(theta)*round(cos(alpha)), -cos(theta)*round(sin(alpha)), a*sin(theta);
                           0, round(sin(alpha)), round(cos(alpha)), d;
                           0, 0, 0, 1];

% Transformation matrices for each joint
A1 = A(theta1, d1, a1, alpha1);
A2 = A(theta2, 0, a2, alpha2);
A3 = A(theta3, 0, a3, alpha3);
A4 = A(theta4, 0, a4, alpha4);
A5i = A(theta4, 0, a5, 0);
Y5 = [1, 0, 0, 0;
    0, 1, 0, d5_y;
    0, 0, 1, 0;
    0, 0, 0, 1];

A5 = A5i * Y5;


% Forward Kinematics: Multiply the matrices to get the end-effector position
T04 = A1 * A2 * A3 * A4; % end effector pos
T05 = A1 * A2 * A3 * A5 ; % camera position

% Display the final transformation matrix

%% Problem 2 
% Inverse Kinematics

% Extract the position vector from the transformation matrix T04
position_o4 = T04(1:3, 4); 

% Define the desired circular trajectory
syms R phi; % Radius and angle parameter for the circular path
p0_c = [0; 0; 0]; % Example center of the circle
phi = 0; % with phi [0:2pi];
R = 0.02;
p0_desired = p0_c + R * [0; cos(phi); sin(phi)];

% Solve for the joint angles such that the position matches the desired trajectory
eqns = position_o4 == p0_desired;

% Now use vpasolve() to solve the system numerically with a range for each variable
theta_guesses = [deg2rad(0), deg2rad(0), deg2rad(0), deg2rad(0)];  % Vector of initial guesses for all angles

solutions = vpasolve(eqns, [theta1, theta2, theta3, theta4], theta_guesses);

% Display the solutions for the joint angles
if isempty(solutions.theta1)
    disp('No solutions found. Adjust the initial guesses or check the desired position.');
else
    % Display the numerical solutions for the joint angles
    theta1_sol = double(solutions.theta1);
    theta2_sol = double(solutions.theta2);
    theta3_sol = double(solutions.theta3);
    theta4_sol = double(solutions.theta4);

    disp('Numerical solution for theta1:');
    disp(theta1_sol);
    disp('Numerical solution for theta2:');
    disp(theta2_sol);
    disp('Numerical solution for theta3:');
    disp(theta3_sol);
    disp('Numerical solution for theta4:');
    disp(theta4_sol);
end

%% Problem 3

% Define constants
R = 0.032; % Radius in meters (32 mm)
pc = [0.150; 0; 0.120]; % Center of the circle in meters (150 mm, 0 mm, 120 mm)
num_points = 37; % 36 points + 1 for the full circle
phi_vals = linspace(0, 2*pi, num_points); % Equidistant points on the circle

% Preallocate for joint angles
q1_vals = zeros(1, num_points);
q2_vals = zeros(1, num_points);
q3_vals = zeros(1, num_points);
q4_vals = zeros(1, num_points); % Assuming q4 remains constant

% Loop through each point on the circle
for j = 1:num_points
    % Define the desired position on the circle
    p_desired = pc + R * [0; cos(phi_vals(j)); sin(phi_vals(j))];
    
    % Solve for the inverse kinematics (q1, q2, q3, q4) such that:
    % The end-effector position matches p_desired
    % Note: You need to write the inverse kinematics function that takes
    % the desired position and calculates the joint angles.
    
    [q1, q2, q3, q4] = inverse_kinematics(p_desired, a2, a3, a4, d1);
    
    % Store the joint angles
    q1_vals(j) = q1;
    q2_vals(j) = q2;
    q3_vals(j) = q3;
    q4_vals(j) = q4; % Keep q4 constant
end

% Display the results
disp('Joint configurations for tracking the circle:');
disp('q1 values:');
disp(q1_vals);
disp('q2 values:');
disp(q2_vals);
disp('q3 values:');
disp(q3_vals);
disp('q4 values:');
disp(q4_vals);

%% Problem 4

joint_types_end_eff = [1; 1; 1; 1]; % 4 joints, all revolutes (1 for revolute and 0 for prismatic)
joint_types_camera = [1; 1; 1; 1]; % we consider the distance camera-end effector as a costant prismatic joint 

A_end_effector = {A1, A2, A3, A4}; % homogeneus transformation matrix array for the end effector

A_camera = {A1, A2, A3, A5}; % homogeneus transformation matrix array for the camera

%now we evaluate the two manipulator jacobians
J_end_eff = manipulatorJacobian(A_end_effector, joint_types_end_eff);
J_camera = manipulatorJacobian(A_camera, joint_types_camera);

%Report the numerical results for the two Jacobians at
% φ = 0, φ = π/2, φ = π, and φ = 3π/2 along the path studies in Problem 3.

% Define the angles phi to be evaluated (0, pi/2, pi, 3pi/2)
phi_values = [0, pi/2, pi, 3*pi/2];
J_end_eff = {0,0,0,0};
J_camera = {0,0,0,0};

% Loop over each value of phi and compute the Jacobians
for i = 1:length(phi_values)
    phi = phi_values(i);
    
    % Set the joint angles for the given phi value
    p0_desired = p0_c + R * [0; cos(phi); sin(phi)];

    % Solve for the joint angles such that the position matches the desired trajectory
    eqns = position_o4 == p0_desired;
    
    theta_guesses = [deg2rad(0), deg2rad(0), deg2rad(0), deg2rad(0)];  % Vector of initial guesses for all angles

    %solutions = vpasolve(eqns, [theta1, theta2, theta3, theta4], theta_guesses);
    [theta1_val, theta2_val, theta3_val, theta4_val] = inverse_kinematics(p_desired, a2, a3, a4, d1);

    % Create a vector of joint configurations q
    q = [theta1_val, theta2_val, theta3_val, theta4_val];
    
    % Substitute numerical values of q into the transformation matrices
    A_end_effector_num = {0,0,0,0};
    A_camera_num = {0,0,0,0};
    for m = 1:length(A_end_effector)
        A_end_effector_num{m} = double(subs(A_end_effector{m}, [theta1, theta2, theta3, theta4], q));
        A_camera_num{m} = double(subs(A_camera{m}, [theta1, theta2, theta3, theta4], q));
    end
    % Compute the Jacobians with the substituted numerical values
    J_end_eff{i} = double(manipulatorJacobian(A_end_effector_num, joint_types_end_eff));
    J_camera{i} = double(manipulatorJacobian(A_camera_num, joint_types_camera));
    
    % Display the results
    disp(['Numerical Jacobian for the end-effector at phi = ', num2str(phi)]);
    disp(J_end_eff);
    
    disp(['Numerical Jacobian for the camera at phi = ', num2str(phi)]);
    disp(J_camera);
end
%% Problem 5

v4_0 = [0, -0.003, 0]; %m/s, it is the desired velocity for the stylo
omega4 = [0, 0, 0]; % Angular velocity with unknown x and y components, z component is zero

% Combine linear and angular velocities into the complete end-effector velocity vector
x_dot_4 = [v4_0, omega4]';

J = J_end_eff{2};

% Use the pinv function (pseudoinverse) to handle cases where J is not square
q_dot = pinv(J) * x_dot_4;

disp('joint velocities')
disp(q_dot)





