function [Xplus Cov Jac1 Jac2] = composition(X1, C1, X2, C2, noise)
%   The actual Composition is done here. Including the calculation 
%   of both Jacobians and the resulting covariances
% INPUT  : X1 absolute state
%          C1 covaraince corresponding to X1
%          X2 relative state
%          C2 covaraince corresponding to X2
%          noise is a flag to perform a noise addition (set) or not (unset)

%   Important Notes:
%       - This version is using quaternions
%       - The states need quaternions and should look like follows:
%           * X = [X, Y, Z, w, x, y, z], where [w, x, y, z] = q

%      put noise to the relative motion this is more likely simulating an
%      error of the odometry
if nargin == 5
%     define sigma matrix
%     sigma for x y z qw q1 q2 q3
    s = 3e-9;
    sigmas = [s s s s s s s]; 
    SIGMA = diag(sigmas);
    X2 = addNoiseToState(X2, SIGMA)';
end

% get quaternions from data
    q1 = [ X1(4), X1(5), X1(6), X1(7) ];
    q2 = [ X2(4), X2(5), X2(6), X2(7) ];
      
% building rotation matrix from quaternion
    R  = quatToMatrix(q1);
    
% building translaten matrix
    t1 = [ X1(1); X1(2); X1(3); 1 ];
    t2 = [ X2(1); X2(2); X2(3); 1 ];
% composition vector (Part I). Here is where the main information is
    comVec = R * t2;
    
% and rotation: Akkumulation of Rotation ==> quaternion multiplication    
    rot    = quatMult(q1, q2);

% putting everything together
    Xplus  = [ t1(1) + comVec(1); ...
               t1(2) + comVec(2); ...
               t1(3) + comVec(3); ...
               rot(1); ...
               rot(2); ...
               rot(3); ...
               rot(4)];
           
     if nargout > 1
         
        x1 = X1(1);
        y1 = X1(2); 
        z1 = X1(3);
        q1 = X1(4);
        q2 = X1(5); 
        q3 = X1(6);
        q4 = X1(7);
        x2 = X2(1); 
        y2 = X2(2); 
        z2 = X2(3); 
        q5 = X2(4); 
        q6 = X2(5); 
        q7 = X2(6); 
        q8 = X2(7); 
         
Jac1 =[
[ 1, 0, 0, 2*q3*z2 - 2*q4*y2,           2*q3*y2 + 2*q4*z2, 2*q2*y2 - 4*q3*x2 + 2*q1*z2, 2*q2*z2 - 2*q1*y2 - 4*q4*x2]
[ 0, 1, 0, 2*q4*x2 - 2*q2*z2, 2*q3*x2 - 4*q2*y2 - 2*q1*z2,           2*q2*x2 + 2*q4*z2, 2*q1*x2 - 4*q4*y2 + 2*q3*z2]
[ 0, 0, 1, 2*q2*y2 - 2*q3*x2, 2*q4*x2 + 2*q1*y2 - 4*q2*z2, 2*q4*y2 - 2*q1*x2 - 4*q3*z2,           2*q2*x2 + 2*q3*y2]
[ 0, 0, 0,                q5,                         -q6,                         -q7,                         -q8]
[ 0, 0, 0,                q6,                          q5,                          q8,                         -q7]
[ 0, 0, 0,                q7,                         -q8,                          q5,                          q6]
[ 0, 0, 0,                q8,                          q7,                         -q6,                          q5]];
 
 
 
Jac2 =[ 
[ - 2*q3^2 - 2*q4^2 + 1,     2*q2*q3 - 2*q1*q4,     2*q1*q3 + 2*q2*q4,  0,   0,   0,   0]
[     2*q1*q4 + 2*q2*q3, - 2*q2^2 - 2*q4^2 + 1,     2*q3*q4 - 2*q1*q2,  0,   0,   0,   0]
[     2*q2*q4 - 2*q1*q3,     2*q1*q2 + 2*q3*q4, - 2*q2^2 - 2*q3^2 + 1,  0,   0,   0,   0]
[                     0,                     0,                     0, q1, -q2, -q3, -q4]
[                     0,                     0,                     0, q2,  q1, -q4,  q3]
[                     0,                     0,                     0, q3,  q4,  q1, -q2]
[                     0,                     0,                     0, q4, -q3,  q2,  q1]];
 
            
         Cov  = Jac1 * C1 * transpose( Jac1 ) + Jac2 * C2 * transpose( Jac2 );
     end
end

%%
function f()
%%
    syms xasdX yasdX zasdX q_wasdX q_1asdX q_2asdX q_3asdX xasdY yasdY zasdY q_wasdY q_1asdY q_2asdY q_3asdY;
    x1 = [ xasdX, yasdX, zasdX, q_wasdX, q_1asdX, q_2asdX, q_3asdX ];
    x2 = [ xasdY, yasdY, zasdY, q_wasdY, q_1asdY, q_2asdY, q_3asdY ];
    cov1  = zeros( 7, 7 );
    cov2  = zeros( 7, 7 );
    [p_r cov] = composition(x1, cov1, x2, cov2);
    p_r;
%     Jac1 = jacobian(p_r, x1)
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