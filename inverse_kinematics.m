function [q1, q2, q3, q4] = inverse_kinematics(p_desired, a2, a3, a4, d1)
    % INPUTS:
    % p_desired: 3x1 desired position of the end-effector [x; y; z]
    % a2: length of the second link
    % a3: length of the third link
    % a4: length of 4 link
    % d1: height offset from the base to the first joint
    
    % Extract the desired coordinates
    x = p_desired(1);
    y = p_desired(2);
    z = p_desired(3);
    l_desired = sqrt(x^2 + y^2 + z^2);

    % Solve for q1 (rotation around the z-axis, in the xy-plane)

    q1 = atan2(y, x);
    r = sqrt(x^2 + y^2);
    s = z - d1;
    alfa = abs(z)/l_desired;
    x_prime = r - a4 * (1 - alfa^2);
    y_prime = s - a4 * alfa;
    
    % Solve for q2 and q3 using the geometric approach
    % Law of cosines to solve for q3
    D = (x_prime^2 + y_prime^2 - a2^2 - a3^2) / (2 * a2 * a3);
    q3 = atan2(sqrt(1 - D^2), D); % Two solutions, choosing the elbow-up
    
    % Law of sines or another geometric relation to solve for q2
    phi = atan2(y_prime, x_prime); % angle of the triangle in the z-r plane
    beta = atan2(a3 * sin(q3), a2 + a3 * cos(q3)); % angle at joint 2

    if q3 > 0
        q2 = phi - beta;
    else
        q2 = phi + beta;
    end
    
    % For q4, we keep the stylus horizontal (end-effector orientation)
    % Assuming horizontal means q4 = 0
    q4 = atan2(s, r) - q2 - q3;
end


