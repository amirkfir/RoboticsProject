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
T04 = simplify(A1 * A2 * A3 * A4); % end effector pos
T05 = simplify(A1 * A2 * A3 * A5); % camera position
test_q = [deg2rad(0),deg2rad(10), deg2rad(15), deg2rad(-5)];
disp(double(subs(T04,[theta1,theta2,theta3,theta4],test_q)))

%% Problem 2 
% Inverse Kinematics

% Extract the position vector from the transformation matrix T04
position_o4 = T04(1:3, 4); 

% Define the desired circular trajectory
syms R phi; % Radius and angle parameter for the circular path
pc = [0.15; 0; 0.12]; % Example center of the circle
phi = 0; % with phi [0:2pi];
R = 0.02;
p_desired = pc + R * [cos(phi); 0; sin(phi)];

[q1,q2,q3,q4] = inverse_kinematics(p_desired,a2,a3,a4,d1);

disp([q1,q2,q3,q4])

%% Problem 3

% Define constants
R = 0.032; % Radius in meters (32 mm)
pc = [0.150; 0; 0.120]; % Center of the circle in meters (150 mm, 0 mm, 120 mm)
num_points = 37; % 36 points + 1 for the full circle
phi_vals = linspace(0, 2*pi, num_points); % Equidistant points on the circle 
% the cirlce lands in the xz plane, so y = 0

% Preallocate for joint angles
q1_vals = zeros(1, num_points);
q2_vals = zeros(1, num_points);
q3_vals = zeros(1, num_points);
q4_vals = zeros(1, num_points); 

% Loop through each point on the circle
for j = 1:num_points
    % Define the desired position on the circle
    p_desired = pc + R * [0; cos(phi_vals(j)); sin(phi_vals(j))];
  
    [q1, q2, q3, q4] = inverse_kinematics(p_desired, a2, a3, a4, d1);
    
    % Store the joint angles
    q1_vals(j) = q1;
    q2_vals(j) = q2;
    q3_vals(j) = q3;
    q4_vals(j) = q4; 
end

%plot the values
figure
plot(1:num_points, q1_vals)
hold on
plot(1:num_points, q2_vals)
hold on
plot(1:num_points, q3_vals)
hold on
plot(1:num_points, q4_vals)
grid on
legend('q1','q2','q3','q4')
xlabel('iterations')
ylabel('joint values')




%% Problem 4

joint_types_end_eff = [1; 1; 1; 1]; % 4 joints, all revolutes (1 for revolute and 0 for prismatic)
joint_types_camera = [1; 1; 1; 1]; 

A_end_effector = {A1, A2, A3, A4}; % homogeneus transformation matrix array for the end effector

A_camera = {A1, A2, A3, A5}; % homogeneus transformation matrix array for the camera

% Define the angles phi to be evaluated (0, pi/2, pi, 3pi/2)
phi_values = [0, pi/2, pi, 3*pi/2];
J_end_eff = {0,0,0,0};
J_camera = {0,0,0,0};

% Loop over each value of phi and compute the Jacobians
for i = 1:length(phi_values)
    phi = phi_values(i);
    
    % Set the joint angles for the given phi value
    p_desired = pc + R * [0; cos(phi); sin(phi)];
    
    [theta1_val, theta2_val, theta3_val, theta4_val] = inverse_kinematics(p_desired, a2, a3, a4, d1);

    A1 = A(theta1_val, d1, a1, alpha1);
    A2 = A(theta2_val, 0, a2, alpha2);
    A3 = A(theta3_val, 0, a3, alpha3);
    A4 = A(theta4_val, 0, a4, alpha4);

    A_end_effector = {A1,A2,A3,A4};
    
    % Compute the Jacobians with the substituted numerical values
    J_end_eff{i} = double(manipulatorJacobian2(A_end_effector,joint_types_end_eff));
    
    % Display the results
    disp(['Numerical Jacobian for the end-effector at phi = ', num2str(phi)]);
    disp(J_end_eff);
    
end
%% Problem 5

% x4 dot is the rate of change of the orientation of the end effector
% x-axis
phi = pi/2;
p_desired = pc + R * [0; cos(phi); sin(phi)];
[q1,q2,q3,q4] = inverse_kinematics(p_desired,a2,a3,a4,d1);
A1 = A(q1, d1, a1, alpha1);
A2 = A(q2, 0, a2, alpha2);
A3 = A(q3, 0, a3, alpha3);
A4 = A(q4, 0, a4, alpha4);
T04_pi2 = A1 * A2 * A3 * A4;

v4_0 = [0, -0.003, 0]; %m/s, it is the desired velocity for the stylo

rx = T04_pi2(1,4); %x-coordinate of the vector that connects the end-effector to the 0 frame
omega4 = [0,0,v4_0(2)/rx];
% Combine linear and angular velocities into the complete end-effector velocity vector
p_dot_4 = [v4_0, omega4]';

J = J_end_eff{2}; % selecting the one at pi/2 

% Use the pinv function (pseudoinverse) to handle cases where J is not square
q_dot =double(pinv(J) * p_dot_4);

disp('joint velocities')
disp(q_dot)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Part 2 Trajectory planning

%% Problem 6 - using get_quintic_coeff

q0_vals = [q1_vals(1), q2_vals(1), q3_vals(1), q4_vals(1)];
q9_vals = [q1_vals(10), q2_vals(10), q3_vals(10), q4_vals(10)];
q18_vals = [q1_vals(19), q2_vals(19), q3_vals(19), q4_vals(19)];
q27_vals = [q1_vals(28), q2_vals(28), q3_vals(28), q4_vals(28)];
q36_vals = [q1_vals(37), q2_vals(37), q3_vals(37), q4_vals(37)];
% all the needed values for the planning, knot points

% Time settings for each segment
t0 = 0;
tf = 2;
time_steps = 100;
t = linspace(t0,tf,time_steps);

% Velocity settings at end-effector
velocities = {[0, 0, 0], [0, -0.027, 0], [0, 0, -0.027], [0, 0.027, 0], [0, 0, 0]};

% Joint values for each segment
joint_vals = {q0_vals, q9_vals, q18_vals, q27_vals, q36_vals};

% Initialize trajectory coefficients
trajectory = struct('q_d', [], 'q_dot_d', [], 'q_ddot_d', []);

% Iterate over each segment
for i = 1:length(joint_vals) - 1
    % Extract start and end joint values
    q_start = joint_vals{i};
    q_end = joint_vals{i + 1};
    v_start = velocities{i};
    v_end = velocities{i + 1};
    
    % Iterate over each joint
    for j = 1:4
        % Get quintic polynomial coefficients for this joint
        [q_d, q_dot_d, q_ddot_d, t] = get_quintic_coeff(...
            q_start(j), norm(v_start), 0, q_end(j), norm(v_end), 0, t0, tf, time_steps);
        
        % Store trajectory data
        trajectory(i).q_d(:, j) = q_d;
        trajectory(i).q_dot_d(:, j) = q_dot_d;
        trajectory(i).q_ddot_d(:, j) = q_ddot_d;
    end
end

% disp
figure
plot(t,trajectory(1).q_d(:,1))
hold on
plot(t,trajectory(1).q_d(:,2))
hold on
plot(t,trajectory(1).q_d(:,3))
hold on
plot(t,trajectory(1).q_d(:,4))
hold on
grid on
legend('q1','q2','q3','q4')
title('joint values in quintic approximation')
%% Problem 7

% Compute end-effector positions over time using forward kinematics
end_effector_positions_q = [];
for j = 1:length(joint_vals) - 1
    for i = 1:length(trajectory(1).q_d(:,1))
    q = [trajectory(j).q_d(i,1), trajectory(j).q_d(i,2), trajectory(j).q_d(i,3), trajectory(j).q_d(i,4)];
    % Calculate end-effector position from forward kinematics function
    
    pos = forward_kinematics(q);  
    end_effector_positions_q = [end_effector_positions_q, pos];
    end
end

theta = linspace(0, 2*pi, 100); % 100 points for a smooth circle
desired_x = 0.15;
desired_y = R * cos(theta);  % Circle center and radius
desired_z = 0.120 + R * sin(theta);
% Plot the actual end-effector path
figure
plot(end_effector_positions_q(2, :), end_effector_positions_q(3, :), 'b', 'LineWidth', 1.5);
hold on;
plot(desired_y, desired_z, 'r--', 'LineWidth', 1.5);

% Labels and legend
xlabel('y Position (mm)');
ylabel('Z Position (mm)');
title('quintic approx End-Effector Path Comparison');
legend('Actual Path', 'Desired Circular Path');
grid on;
axis equal;

%knot points: (same as problem 6)
q0_vals = [q1_vals(1), q2_vals(1), q3_vals(1), q4_vals(1)];
q9_vals = [q1_vals(10), q2_vals(10), q3_vals(10), q4_vals(10)];
%q13_vals = [q1_vals(14), q2_vals(14), q3_vals(14), q4_vals(14)]; % added trying to imporve
q18_vals = [q1_vals(19), q2_vals(19), q3_vals(19), q4_vals(19)];
%q21_vals = [q1_vals(22), q2_vals(22), q3_vals(22), q4_vals(22)];
q27_vals = [q1_vals(28), q2_vals(28), q3_vals(28), q4_vals(28)];
q36_vals = [q1_vals(37), q2_vals(37), q3_vals(37), q4_vals(37)];


% Define time intervals for the entire trajectory with finer resolution
t_full = linspace(0, 8, 101);  % 40 points for smoothness over full trajectory

% Use spline interpolation for each joint over the knot points
% returns a vector of interpolated values s corresponding to the query points in xq. 
%The values of s are determined by cubic spline interpolation of x and y.
%[q0_vals(1), q9_vals(1), q13_vals(1), q18_vals(1), q21_vals(1), q27_vals(1), q36_vals(1)]
q1_spline = spline([0, 2, 4, 6, 8], [q0_vals(1), q9_vals(1), q18_vals(1), q27_vals(1), q36_vals(1)], t_full);
q2_spline = spline([0, 2, 4, 6, 8], [q0_vals(2), q9_vals(2), q18_vals(2), q27_vals(2), q36_vals(2)], t_full);
q3_spline = spline([0, 2, 4, 6, 8], [q0_vals(3), q9_vals(3), q18_vals(3), q27_vals(3), q36_vals(3)], t_full);
q4_spline = spline([0, 2, 4, 6, 8], [q0_vals(4), q9_vals(4), q18_vals(4), q27_vals(4), q36_vals(4)], t_full);

% Compute end-effector positions over time using forward kinematics
end_effector_positions = [];
for i = 1:length(t_full)
    q = [q1_spline(i); q2_spline(i); q3_spline(i); q4_spline(i)];
    pos = forward_kinematics(q);  % Forward kinematics function
    end_effector_positions = [end_effector_positions, pos];
end

%plot the change of the q_values over time
figure
plot(t_full,q1_spline)
hold on 
plot(t_full,q2_spline)
hold on
plot(t_full,q3_spline)
hold on
plot(t_full,q4_spline)
grid on
legend('q1','q2','q3','q4')
title('angle-time graph of cubic trajectory planning')

% Plot the cubic actual end-effector path
figure
plot(end_effector_positions(2, :), end_effector_positions(3, :), 'b', 'LineWidth', 1.5);
hold on;

% Plot the desired circular path
plot(desired_y, desired_z, 'r--', 'LineWidth', 1.5);

% Labels and legend
xlabel('y Position (mm)');
ylabel('Z Position (mm)');
title('cubic End-Effector Path Comparison');
legend('Actual Path', 'Desired Circular Path');
grid on;
axis equal;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Singularities and statics
% Problem 8

phi_values = linspace(0,2*pi,37);
cond_J = zeros(length(phi_values),1);

q1_vals_true = zeros(length(end_effector_positions(1,:)),1);
q2_vals_true = zeros(length(end_effector_positions(1,:)),1);
q3_vals_true = zeros(length(end_effector_positions(1,:)),1);
q4_vals_true = zeros(length(end_effector_positions(1,:)),1);

% loop for the actual path
for i = 1:length(end_effector_positions(1,:))
    p_desired = end_effector_positions(:,i);

    [theta1_val, theta2_val, theta3_val, theta4_val] = inverse_kinematics(p_desired, a2, a3, a4, d1);
    q1_vals_true(i) = theta1_val;
    q2_vals_true(i) = theta2_val;
    q3_vals_true(i) = theta3_val;
    q4_vals_true(i) = theta4_val;

    A1 = A(theta1_val, d1, a1, alpha1);
    A2 = A(theta2_val, 0, a2, alpha2);
    A3 = A(theta3_val, 0, a3, alpha3);
    A4 = A(theta4_val, 0, a4, alpha4);

    A_end_effector = {A1,A2,A3,A4};
    
    % Compute the Jacobians with the substituted numerical values
    J_end_eff = double(manipulatorJacobian2(A_end_effector,joint_types_end_eff));

    cond_J(i) = double(vpa(cond(J_end_eff))); 

end

[m_cond,idx_m] = max(cond_J);
disp(['closer to a singularity for iteration = ', num2str(idx_m)])

%loop for the desired path
cond_J = zeros(length(phi_values),1);

for i = 1:length(phi_values)
    phi = phi_values(i);
    
    % Set the joint angles for the given phi value
    p_desired = pc + R * [0; cos(phi); sin(phi)];

    %solutions = vpasolve(eqns, [theta1, theta2, theta3, theta4], theta_guesses);
    [theta1_val, theta2_val, theta3_val, theta4_val] = inverse_kinematics(p_desired, a2, a3, a4, d1);

    A1 = A(theta1_val, d1, a1, alpha1);
    A2 = A(theta2_val, 0, a2, alpha2);
    A3 = A(theta3_val, 0, a3, alpha3);
    A4 = A(theta4_val, 0, a4, alpha4);

    A_end_effector = {A1,A2,A3,A4};
    
    % Compute the Jacobians with the substituted numerical values
    J_end_eff = double(manipulatorJacobian2(A_end_effector,joint_types_end_eff));

    cond_J(i) = vpa(cond(J_end_eff));

end

[m_cond,idx_m] = max(cond_J);
disp(['closer to a singularity for phi = ', num2str(phi_values(idx_m))])

%% Problem 9

% we are still assuming the stylo horizontal

%initializing the torques vectors
t1 = zeros(length(phi_values),1);
t2 = zeros(length(phi_values),1);
t3 = zeros(length(phi_values),1);
t4 = zeros(length(phi_values),1);
%forces/torques applied to the end effector wrt world frame
F = [0,0,-1,0,0,0]';

for i=1:length(phi_values)
    phi = phi_values(i);
    
    % Set the joint angles for the given phi value
    p_desired = pc + R * [0; cos(phi); sin(phi)];

    [theta1_val, theta2_val, theta3_val, theta4_val] = inverse_kinematics(p_desired, a2, a3, a4, d1);

    A1 = A(theta1_val, d1, a1, alpha1);
    A2 = A(theta2_val, 0, a2, alpha2);
    A3 = A(theta3_val, 0, a3, alpha3);
    A4 = A(theta4_val, 0, a4, alpha4);

    A_end_effector = {A1,A2,A3,A4};
    T04 = A1*A2*A3*A4;
    
    % Compute the Jacobians with the substituted numerical values
    J_end_eff = double(manipulatorJacobian2(A_end_effector,joint_types_end_eff));

    F_world = T04(1:3,1:3) * F(1:3);

    F = [F_world;0;0;0];

    t = J_end_eff.' * F;

    t1(i) = t(1);
    t2(i) = t(2);
    t3(i) = t(3);
    t4(i) = t(4);

end

%plot the values
figure
plot(1:length(phi_values), t1)
hold on
plot(1:length(phi_values), t2)
hold on
plot(1:length(phi_values), t3)
hold on
plot(1:length(phi_values), t4)
grid on
legend('t1','t2','t3','t4')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Dynamics
% Problem 10 
%we are considering also inertia link 1: rectangular prism
% rotating around an axis parallel to its length link 2-3-4: rectangular
% prisms rotating around an axis perpendicular to their length
% we assume the length along the z axis, as width for the basis we use
w = 0.047; %m, based on onsite evaluation
% as height (lenght of the base):
h = 0.024; %m

% Gravitational acceleration
g_val = 9.81;

% Define other necessary parameters (masses, COMs, inertia tensors)
m1 = 0.06;  % mass of link 1 in kg
m2 = 0.08;  % mass of link 2 
m3 = 0.08;  % mass of link 3 
m4 = 0.04;  % mass of link 4 
m = [m1,m2,m3,m4];

% Center of mass positions in the local link frames 
COM1 = [0; 0; -0.020];
COM2 = [0; 0; -0.030];
COM3 = [0; 0; -0.030];
COM4 = [0; 0; -0.025];

% rough estimations of the inertia tensor
I1 = 1/12 * m1 * (a1^2 + w^2); 
I2 = 1/12 * m2 * (a2^2 + h^2);
I4 = 1/12 * m4 * (a4^2 + h^2);

% inertia tensors
I_tens1 = [I1, 0, 0; 0, 0.4*I1, 0; 0, 0, 0.9*I1];
I_tens2 = [0.45*I2, 0, 0; 0, 1.4*I2, 0; 0, 0, 1.2*I2];
I_tens3 = I_tens2;
I_tens4 = [0.5*I4, 0, 0; 0, 0.5*I4, 0; 0, 0, 0.5*I4];

syms q1 q2 q3 q4
q = [q1,q2,q3,q4];
% Transformation matrices from the base frame to each link
T01 = A(q1,d1, a1, alpha1);  
T02 = T01 * A(q2,0,a2,0);
T03 = T02 * A(q3,0,a3,0);
T04 = T03 * A(q4,0,a4,0);

% Extract rotation matrices from transformations
R01 = T01(1:3, 1:3);
R02 = T02(1:3, 1:3);
R03 = T03(1:3, 1:3);
R04 = T04(1:3, 1:3);

% Extract positions of centers of mass in the base frame
p1 = T01 * [COM1; 1];
p2 = T02 * [COM2; 1];
p3 = T03 * [COM3; 1];
p4 = T04 * [COM4; 1];

% Only need the first three components for position vectors
% they are the Rci viewed in the theory
p1 = p1(1:3); 
p2 = p2(1:3);
p3 = p3(1:3);
p4 = p4(1:3);

% Inertia tensors of each center of mass link with respect to the base frame
I1 = R01 * I_tens1 * R01' ;
I2 = R02 * I_tens2 * R02' ;
I3 = R03 * I_tens3 * R03' ;
I4 = R04 * I_tens4 * R04' ;

% Calculate the inertia matrix by summing contributions from each link

% Define Jacobians for linear and angular velocities for each link's COM
A_matrices = {A1,A2,A3,A4};

Jv = center_of_mass_velocity_jacobian(p1,p2,p3,p4,q1,q2,q3,q4); 
J = double(manipulatorJacobian2(A_matrices,joint_types_end_eff));
Jw = J(4:6,:);

% Initialize the inertia matrix
D = sym(zeros(4, 4));

% Loop over each link/joint
for i = 1:4
    % Linear velocity Jacobian for joint i
    Jv_i = Jv(:, i); % Extract the ith column of J_v
    
    % Angular velocity Jacobian for joint i
    Jw_i = Jw(:, i); % Extract the ith column of J_w (assume similar structure)
    
    % Inertia tensor for link i in the base frame
    I_i = eval(['I' num2str(i)]); % Retrieve I1, I2, I3, or I4 dynamically
    
    % Add contributions of the current link to the D matrix
    D = D + m(i) * (Jv_i' * Jv_i) + Jw_i' * I_i * Jw_i;
end
D = simplify(D);

C = sym(zeros(4, 4));

% Calculate each element of the Coriolis matrix
for i = 1:4
    for j = 1:4
        Cij = 0;
        for k = 1:4
            % Calculate Christoffel symbols and accumulate for each C(i,j)
            Cij = Cij + (1/2) * (diff(D(i,j), q(k)) + diff(D(i,k), q(j)) - diff(D(j,k), q(i)));
        end
        C(i,j) = simplify(Cij);
    end
end

% Define the potential energy of each link due to gravity
P1 = m1 * g_val * p1(3);  % height is along the z-axis
P2 = m2 * g_val * p2(3);
P3 = m3 * g_val * p3(3);
P4 = m4 * g_val * p4(3);

% Total potential energy
P_total = P1 + P2 + P3 + P4;

% Gravity vector g(q) is the gradient of the potential energy w.r.t. joint angles
g = [diff(P_total, q1);
     diff(P_total, q2);
     diff(P_total, q3);
     diff(P_total, q4)];

% Simplify and display the gravity vector
g = simplify(g);
disp('Gravity vector g(q):');
disp(g);
%%
% we need the values of q, q_dot and q_dotdot

figure
q_d = {0,0,0,0};
q_dot_d = {0,0,0,0};
q_ddot_d = {0,0,0,0};
for i=1:4
    joint_val_st = joint_vals{i};
    joint_val_fin = joint_vals{i+1};

    % Define known joint positions for q1 
    q1_start = joint_val_st(1); % Joint 1 position 
    q1_end = joint_val_fin(1); % Joint 1 position 
    % Define known joint positions for q2 
    q2_start = joint_val_st(2); % Joint 2 position 
    q2_end = joint_val_fin(2); % Joint 2 position 
    % Define known joint positions for q3 
    q3_start = joint_val_st(3); % Joint 3 position 
    q3_end = joint_val_fin(3); % Joint 3 position 
    % Define known joint positions for q4 
    q4_start = joint_val_st(4); % Joint 4 position 
    q4_end = joint_val_fin(4); % Joint 4 position 

    q_start = [q1_start, q2_start, q3_start, q4_start];
    q_end = [q1_end, q2_end, q3_end, q4_end];

    [q_d{i}, q_dot_d{i}, q_ddot_d{i}, t]= get_quintic_coeff(q_start(i),norm(velocities{i}),0,q_end(i),norm(velocities{i+1}), ...
        0,0,8,37);
    % we are assuming 8 seconds in total to get to all the 37
    % points in the circular path

    subplot(3,1,1)
    title('joint values')
    plot(t,q_d{i})
    xlabel('seconds')
    ylabel('rad')
    hold on
    subplot(3,1,2)
    title('joint velocity values')
    plot(t,q_dot_d{i})
    xlabel('seconds')
    ylabel('rad/s')
    hold on
    subplot(3,1,3)
    title('joint acceleration values')
    plot(t,q_ddot_d{i})
    xlabel('seconds')
    ylabel('rad/s^2')
    hold on

end
%%
num_points = length(q_d{1});

% Preallocate arrays for torques
tau1_vals = zeros(1, num_points);
tau2_vals = zeros(1, num_points);
tau3_vals = zeros(1, num_points);
tau4_vals = zeros(1, num_points);

% Calculate torques for each point in the trajectory
for i = 1:num_points
    q = [q_d{1}(i),q_d{2}(i),q_d{3}(i),q_d{4}(i)];
    q_dot = [q_dot_d{1}(i),q_dot_d{2}(i),q_dot_d{3}(i),q_dot_d{4}(i)];
    q_ddot = [q_ddot_d{1}(i),q_ddot_d{2}(i),q_ddot_d{3}(i),q_ddot_d{4}(i)];
    
    % Compute the required torque
    tau = double(subs(D,[q1,q2,q3,q4],q)) * q_ddot' + double(subs(C,[q1,q2,q3,q4],q)) * q_dot' + double(subs(g,[q1,q2,q3,q4],q));
    tau1_vals(i) = tau(1);
    tau2_vals(i) = tau(2);
    tau3_vals(i) = tau(3);
    tau4_vals(i) = tau(4);
end

% Plot the joint torques as functions of time or trajectory position
figure;
plot(1:num_points, tau1_vals, 'r', 'DisplayName', '\tau_1');
hold on;
plot(1:num_points, tau2_vals, 'g', 'DisplayName', '\tau_2');
plot(1:num_points, tau3_vals, 'b', 'DisplayName', '\tau_3');
plot(1:num_points, tau4_vals, 'm', 'DisplayName', '\tau_4');
hold off;

% Configure plot
xlabel('Trajectory iteration');
ylabel('Joint Torque (Nm)');
title('Joint Torques Required for Following the Trajectory');
legend;
grid on;

