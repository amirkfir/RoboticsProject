function Jvci = center_of_mass_velocity_jacobian(r1,r2,r3,r4,q1,q2,q3,q4)
%r1,r2,r3,r4 are positions of centers of mass in the base frame
% they are function of q1,q2,q3,q4
Jvci = sym(zeros(4,4));
r = [r1,r2,r3,r4];
q = [q1,q2,q3,q4];

for i=1:4
    for j=1:4
        Jvci(i,j) = diff(r(i),q(j));
    end
end




