function [Xminus cov] = inversion(X, C)
%   This function calculates the 3D inversion using quaternions.
%   Note:   p is the translation vector (easy as it gets)
%           n, o and a are the column vector of the 3D Rotation Matrix.
%           Have a look on the pdf in the "doc" subfolder for more details.

% building translation matrix
    p = [ X(1); X(2); X(3); 1];

% get quaternions from data
    q = [ X(4), X(5), X(6), X(7) ];
      
% building rotation matrix
    R  = quatToMatrix(q);

% inverting
    qinv = quatInvers(q);
     
    n = R(:,1);
    
    o = R(:,2);
      
    a = R(:,3);
      
    Xminus = [ dot(-n, p);
               dot(-o, p);
               dot(-a, p);
               -qinv' ];
               
     if nargout > 1
        x1 = X(1);
        y1 = X(2); 
        z1 = X(3);
        q1 = X(4);
        q2 = X(5); 
        q3 = X(6);
        q4 = X(7);
        
       Jac =[ 
[             2*conj(q3)^2 + 2*conj(q4)^2 - 1, - 2*conj(q1)*conj(q4) - 2*conj(q2)*conj(q3),   2*conj(q1)*conj(q3) - 2*conj(q2)*conj(q4), 2*z1*conj(q3) - 2*y1*conj(q4),               - 2*y1*conj(q3) - 2*z1*conj(q4), 4*x1*conj(q3) - 2*y1*conj(q2) + 2*z1*conj(q1), 4*x1*conj(q4) - 2*y1*conj(q1) - 2*z1*conj(q2)]
[   2*conj(q1)*conj(q4) - 2*conj(q2)*conj(q3),             2*conj(q2)^2 + 2*conj(q4)^2 - 1, - 2*conj(q1)*conj(q2) - 2*conj(q3)*conj(q4), 2*x1*conj(q4) - 2*z1*conj(q2), 4*y1*conj(q2) - 2*x1*conj(q3) - 2*z1*conj(q1),               - 2*x1*conj(q2) - 2*z1*conj(q4), 2*x1*conj(q1) + 4*y1*conj(q4) - 2*z1*conj(q3)]
[ - 2*conj(q1)*conj(q3) - 2*conj(q2)*conj(q4),   2*conj(q1)*conj(q2) - 2*conj(q3)*conj(q4),             2*conj(q2)^2 + 2*conj(q3)^2 - 1, 2*y1*conj(q2) - 2*x1*conj(q3), 2*y1*conj(q1) - 2*x1*conj(q4) + 4*z1*conj(q2), 4*z1*conj(q3) - 2*y1*conj(q4) - 2*x1*conj(q1),               - 2*x1*conj(q2) - 2*y1*conj(q3)]
[                                           0,                                           0,                                           0,                            -1,                                             0,                                             0,                                             0]
[                                           0,                                           0,                                           0,                             0,                                             1,                                             0,                                             0]
[                                           0,                                           0,                                           0,                             0,                                             0,                                             1,                                             0]
[                                           0,                                           0,                                           0,                             0,                                             0,                                             0,                                             1]];
 
         cov = Jac * C * transpose( Jac );
  
     end

end

%%
function f()
%%
    syms xasdX yasdX zasdX q_wasdX q_1asdX q_2asdX q_3asdX;
    x1 = [ xasdX, yasdX, zasdX, q_wasdX, q_1asdX, q_2asdX, q_3asdX ];
    cov = zeros( 7,7 );
    p_r = inversion(x1, cov);
    Jac = jacobian(p_r, x1)
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