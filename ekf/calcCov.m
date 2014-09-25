function C = calcCov(  C, Cnew, Jac1, Jac2 )
% This function builds the big covariance used in EKF
% INPUT:  C is the covariance Matrix from the previous iteration
%         Cnew is the covariance from the current state augmentation
%         Jac1 Jacobian of the composition with respect to the last item of
%              the state vector
%         Jac2 Jacobian of the composition with respect to the odometry
% Output: C the new covariance matrix

% C will be build as follows:
% C = | C(1,1)      C(1,2)      ... C(1,n)      C(1,n)*Jac1' |
%     | C(2,1)      C(2,2)      ... C(2,n)      C(2,n)*Jac1' |
%     |                         ...                          |
%     | C(m,1)      C(m,2)      ... C(m,n)      C(m,n)*Jac1' |
%     | Jac1*C(m,1) Jac1*C(m,2) ... Jac1*C(m,n) Cnew         |

sizeC = length(C)/7;

% Pushing the new covariance to C
    C( (sizeC+1)*7-6:(sizeC+1)*7, (sizeC+1)*7-6:(sizeC+1)*7 ) = Cnew;

% % Put information to the last column and row of C
    for i=1:sizeC
%     column --> C(i,n+1) = C(i,n)*Jac1'
        C( i*7-6:i*7, (sizeC+1)*7-6:(sizeC+1)*7 ) = C( i*7-6:i*7, sizeC*7-6:sizeC*7 ) ...
                                                                   * Jac1';

%     row -----> C(m+1,i) = Jac1*C(m,1)
        C( (sizeC+1)*7-6:(sizeC+1)*7, i*7-6:i*7 ) = Jac1 * ...
                                          C( sizeC*7-6:sizeC*7, i*7-6:i*7);
    end
    
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