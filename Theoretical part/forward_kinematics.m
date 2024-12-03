function end_effector_pos = forward_kinematics(q)
    % Forward Kinematics function for a 4-DOF robotic arm
    % q: 4x1 vector of joint angles in radians
    % end_effector_pos: 3x1 vector of end-effector position [x; y; z]

    % Define DH parameters for each joint
    % These values should be set according to the specific robotic arm configuration
    DH_params = [
        q(1), 0.05, 0, pi/2;  % Joint 1
        q(2), 0, 0.093, 0;       % Joint 2
        q(3), 0, 0.093, 0;   % Joint 3
        q(4), 0, 0.05, 0        % Joint 4 (end-effector link)
    ];
    
    % Initialize transformation matrix as identity matrix
    T = eye(4);
    
    % Loop through each joint to compute the transformation matrix
    for i = 1:4
        theta = DH_params(i, 1);
        d = DH_params(i, 2);
        a = DH_params(i, 3);
        alpha = DH_params(i, 4);
        
        % Compute the transformation matrix for this joint
        T_i = [cos(theta), -sin(theta)*double(cos(alpha)), sin(theta)*double(sin(alpha)), a*cos(theta);
               sin(theta), cos(theta)*double(cos(alpha)), -cos(theta)*double(sin(alpha)), a*sin(theta);
               0,          double(sin(alpha)),             double(cos(alpha)),            d;
               0,          0,                      0,                     1];
        
        % Multiply the transformation matrices
        T = T * T_i;
    end
    
    % Extract the position of the end-effector from the final transformation matrix
    end_effector_pos = T(1:3, 4);
end

