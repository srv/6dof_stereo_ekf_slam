function [R, cov, Jac1, Jac2] = relativeMotionFromAbsoluteMotionUQ(A1, cov1, A2, cov2)
%   This function calculates the 3D relative motion between states
%   from two given 3D absolute state vectors.
%   Note:   R = opMINUS(A1) opPLUS(A2)
%           Provide the absolute state as follows:
%               - [ x, y, z, A1(4), A1(5), A1(6), A1(7) ]
%               - where q_n defines a quaternion
%           Have a look on the pdf in the "doc" subfolder for more details.

    [invA1 covIn] = inversion(A1, cov1);
    [R cov] = composition(invA1, covIn, A2, cov2);
       
     if nargout > 2
        x1 = A1(1);
        y1 = A1(2); 
        z1 = A1(3);
        q1 = A1(4);
        q2 = A1(5); 
        q3 = A1(6);
        q4 = A1(7);
        x2 = A2(1); 
        y2 = A2(2); 
        z2 = A2(3); 
        q5 = A2(4); 
        q6 = A2(5); 
        q7 = A2(6); 
        q8 = A2(7);         
         
        Jac1 = [
[             2*conj(q3)^2 + 2*conj(q4)^2 - 1, - 2*conj(q1)*conj(q4) - 2*conj(q2)*conj(q3),   2*conj(q1)*conj(q3) - 2*conj(q2)*conj(q4), 2*y2*conj(q4) - 2*y1*conj(q4) + 2*z1*conj(q3) - 2*z2*conj(q3),                                 2*y2*conj(q3) - 2*y1*conj(q3) - 2*z1*conj(q4) + 2*z2*conj(q4), 4*x1*conj(q3) - 4*x2*conj(q3) - 2*y1*conj(q2) + 2*y2*conj(q2) + 2*z1*conj(q1) - 2*z2*conj(q1), 4*x1*conj(q4) - 4*x2*conj(q4) - 2*y1*conj(q1) + 2*y2*conj(q1) - 2*z1*conj(q2) + 2*z2*conj(q2)]
[   2*conj(q1)*conj(q4) - 2*conj(q2)*conj(q3),             2*conj(q2)^2 + 2*conj(q4)^2 - 1, - 2*conj(q1)*conj(q2) - 2*conj(q3)*conj(q4), 2*x1*conj(q4) - 2*x2*conj(q4) - 2*z1*conj(q2) + 2*z2*conj(q2), 2*x2*conj(q3) - 2*x1*conj(q3) + 4*y1*conj(q2) - 4*y2*conj(q2) - 2*z1*conj(q1) + 2*z2*conj(q1),                                 2*x2*conj(q2) - 2*x1*conj(q2) - 2*z1*conj(q4) + 2*z2*conj(q4), 2*x1*conj(q1) - 2*x2*conj(q1) + 4*y1*conj(q4) - 4*y2*conj(q4) - 2*z1*conj(q3) + 2*z2*conj(q3)]
[ - 2*conj(q1)*conj(q3) - 2*conj(q2)*conj(q4),   2*conj(q1)*conj(q2) - 2*conj(q3)*conj(q4),             2*conj(q2)^2 + 2*conj(q3)^2 - 1, 2*x2*conj(q3) - 2*x1*conj(q3) + 2*y1*conj(q2) - 2*y2*conj(q2), 2*x2*conj(q4) - 2*x1*conj(q4) + 2*y1*conj(q1) - 2*y2*conj(q1) + 4*z1*conj(q2) - 4*z2*conj(q2), 2*x2*conj(q1) - 2*x1*conj(q1) - 2*y1*conj(q4) + 2*y2*conj(q4) + 4*z1*conj(q3) - 4*z2*conj(q3),                                 2*x2*conj(q2) - 2*x1*conj(q2) - 2*y1*conj(q3) + 2*y2*conj(q3)]
[                                           0,                                           0,                                           0,                                                           -q5,                                                                                           -q6,                                                                                           -q7,                                                                                           -q8]
[                                           0,                                           0,                                           0,                                                           -q6,                                                                                            q5,                                                                                            q8,                                                                                           -q7]
[                                           0,                                           0,                                           0,                                                           -q7,                                                                                           -q8,                                                                                            q5,                                                                                            q6]
[                                           0,                                           0,                                           0,                                                           -q8,                                                                                            q7,                                                                                           -q6,                                                                                            q5]];
 
         
        Jac2 = [
[           1 - 2*conj(q4)^2 - 2*conj(q3)^2, 2*conj(q1)*conj(q4) + 2*conj(q2)*conj(q3), 2*conj(q2)*conj(q4) - 2*conj(q1)*conj(q3),         0,         0,         0,         0]
[ 2*conj(q2)*conj(q3) - 2*conj(q1)*conj(q4),           1 - 2*conj(q4)^2 - 2*conj(q2)^2, 2*conj(q1)*conj(q2) + 2*conj(q3)*conj(q4),         0,         0,         0,         0]
[ 2*conj(q1)*conj(q3) + 2*conj(q2)*conj(q4), 2*conj(q3)*conj(q4) - 2*conj(q1)*conj(q2),           1 - 2*conj(q3)^2 - 2*conj(q2)^2,         0,         0,         0,         0]
[                                         0,                                         0,                                         0, -conj(q1), -conj(q2), -conj(q3), -conj(q4)]
[                                         0,                                         0,                                         0,  conj(q2), -conj(q1), -conj(q4),  conj(q3)]
[                                         0,                                         0,                                         0,  conj(q3),  conj(q4), -conj(q1), -conj(q2)]
[                                         0,                                         0,                                         0,  conj(q4), -conj(q3),  conj(q2), -conj(q1)]];
 
     end
    
end

%%
function f()
%%
    syms x1 y1 z1 q1 q2 q3 q4 x2 y2 z2 q5 q6 q7 q8;
    x1 = [ x1, y1, z1, q1, q2, q3, q4 ];
    x2 = [ x2, y2, z2, q5, q6, q7, q8 ];
    cov1  = zeros( 7, 7 );
    cov2  = zeros( 7, 7 );
    [p_r cov] = relativeMotionFromAbsoluteMotionUQ(x1, cov1, x2, cov2);
    Jac1 = jacobian(p_r, x1)
    Jac2 = jacobian(p_r, x2)
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Copyright (c) 2014, Markus Solbach
% All rights reserved.

% Redistribution and use in source and binary forms, with or without
% modification, are permitted provided that the following conditions are
% met:

%     * Redistributions of source code must retain the above copyright
%       notice, this list of conditions and the following disclaimer.
%     * Redistributions in binary form must reproduce the above copyright
%       notice, this list of conditions and the following disclaimer in
%       the documentation and/or other materials provided with the distribution

% THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
% AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
% IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
% ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
% LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
% CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
% SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
% INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
% CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
% ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
% POSSIBILITY OF SUCH DAMAGE.