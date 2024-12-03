function J = manipulatorJacobian(q)
    % Define the DH parameters based on input joint angles
    DH_params = [
        q(1), 0.05, 0, pi/2;  % Joint 1
        q(2), 0, 0.093, 0;    % Joint 2
        q(3), 0, 0.093, 0;    % Joint 3
        q(4), 0, 0.05, 0      % Joint 4 (end-effector link)
    ];

    % Number of joints
    num_joints = size(DH_params, 1);

    % Initialize transformation matrix and Jacobian matrix
    T = eye(4);
    J = zeros(6, num_joints); % 6x4 Jacobian for a 3D robotic arm with 4 joints

    % Store positions and z-axes of each frame
    positions = zeros(3, num_joints + 1); % Position of each joint
    z_axes = zeros(3, num_joints + 1);    % Z-axis of each joint

    % Initial base position and Z-axis
    positions(:, 1) = [0; 0; 0];
    z_axes(:, 1) = [0; 0; 1];

    % Compute the transformation matrix for each joint and store intermediate results
    for i = 1:num_joints
        theta = DH_params(i, 1);
        d = DH_params(i, 2);
        a = DH_params(i, 3);
        alpha = DH_params(i, 4);

        % Compute the A matrix for each joint based on DH parameters
        A = [
            cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta);
            sin(theta), cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
            0, sin(alpha), cos(alpha), d;
            0, 0, 0, 1
        ];

        % Update the overall transformation matrix
        T = T * A;

        % Extract the position and Z-axis for the current joint
        positions(:, i + 1) = T(1:3, 4);    % Position of the end of the current link
        z_axes(:, i + 1) = T(1:3, 3);       % Z-axis of the current joint
    end

    % Calculate the Jacobian
    end_effector_pos = positions(:, end);
    for j = 1:num_joints
        % Position vector from joint j to end-effector
        p = end_effector_pos - positions(:, j);

        % Compute the linear velocity component (cross product of z-axis and position vector)
        J(1:3, j) = cross(z_axes(:, j), p);

        % Compute the angular velocity component (z-axis of the joint)
        J(4:6, j) = z_axes(:, j);
    end
end
