function qNew = quatRearrange(q)
% This function is just a wrapper to use the build in quaternion function
% of Matlab with quaternions which have the scalar value at the last
% instead of the first position
% INPUT  : q as follows: q = [ q1 q2 q3 qw ]
% OUTPUT : qNew as follows: qNew = [ qw q1 q2 q3 ]

    qNew( 1 ) = q(4);
    qNew(2:4) = q(1:3);
    
end