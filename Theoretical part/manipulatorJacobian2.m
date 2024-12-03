function J = manipulatorJacobian2(A_matrices, joint_types)
    % Computes the manipulator Jacobian given the transformation matrices
    % A_matrices: cell array of transformation matrices A1, A2, ..., An
    % joint_types: a vector where 1 represents a revolute joint, 0 represents a prismatic joint
    
    % Number of joints
    n = length(A_matrices);
    
    % Initialize the Jacobian matrix (6 x n)
    J = sym(zeros(6, n));
    
    % Extract the position and orientation of each frame
    o_n = A_matrices{end}(1:3, 4); % End-effector position
    
    % Define the base frame z_0 and o_0
    z_prev = [0; 0; 1]; % Base z-axis (z_0)
    o_prev = [0; 0; 0]; % Base origin (o_0)
    
    % Loop through each joint to compute the Jacobian columns
    for i = 1:n
        % Extract the position of the current joint
        A_prev = A_matrices{i}; % Transformation matrix for joint i
        o_curr = A_prev(1:3, 4); % Position of joint i (o_{i-1})
        z_curr = A_prev(1:3, 3); % z-axis of joint i (z_{i-1})

        % If the joint is revolute (1), calculate the Jacobian for revolute joint
        if joint_types(i) == 1
            % Linear velocity part: Jv_i = z_{i-1} x (o_n - o_{i-1})
            Jv = cross(z_curr, (o_n - o_curr));
            % Angular velocity part: Jw_i = z_{i-1}
            Jw = z_curr;
        else
            % For prismatic joint (0)
            % Linear velocity part: Jv_i = z_{i-1}
            Jv = z_curr;
            % Angular velocity part: Jw_i = [0; 0; 0] (no rotational motion)
            Jw = [0; 0; 0];
        end
        
        % Assign to the Jacobian matrix
        J(1:3, i) = Jv;  % Linear velocity component
        J(4:6, i) = Jw;  % Angular velocity component
    end
end
