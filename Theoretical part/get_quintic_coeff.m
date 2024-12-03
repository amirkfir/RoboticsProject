function [q_d, q_dot_d, q_ddot_d, t] = get_quintic_coeff(q0,v0,ac0,q1,v1,ac1,t0,tf, time_steps)

    
    t = linspace(t0, tf, time_steps+1);
    disp(t)
    
    M = ([
        [1, t0, t0 ^ 2, t0 ^ 3, t0 ^ 4, t0 ^ 5],
        [0, 1, 2*t0, 3*t0 ^ 2, 4*t0 ^ 3, 5*t0 ^ 4],
        [0, 0, 2, 6*t0, 12*t0 ^ 2, 20*t0 ^ 3],
        [1, tf, tf ^ 2, tf ^ 3, tf ^ 4, tf ^ 5],
        [0, 1, 2 * tf, 3*tf ^ 2, 4*tf ^ 3, 5*tf ^ 4],
        [0, 0, 2, 6*tf, 12*tf^2, 20*tf^3]]);
    
    %print('M: \n', M)
    b= [q0, v0, ac0, q1, v1, ac1]';
    disp(pinv(M))
    disp(b)
    a = pinv(M) * b;
    a = round(a, 8);

    % Use of it
    q_d = a(1) + a(2) *t +a(3) *(t .^ 2) + a(4) * (t .^ 3) +a(5)*(t .^ 4) + a(6) *(t .^ 5) ;% position trajectory
    q_dot_d = a(2) + 2 * a(3) * t + 3 *a(4) * t .^ 2 + 4 * a(5) * t .^ 3 + 5 * a(6) * t .^ 4 ;%  velocity trajectory
    q_ddot_d = 2 * a(3) + 6 * a(4) * t + 12 * a(5) * t .^ 2 + 20 * a(6) * t .^ 3 ;%  acceleration trajectory
end