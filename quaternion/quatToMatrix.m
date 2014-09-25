function A = quatToMatrix(q)
% Util program to convert a quaternion to a rotation-matrix
% Remember quaternion representation is as follows: q = [ qw q2 q3 q4]
    qw = q(1);
    qx = q(2); 
    qy = q(3);
    qz = q(4);

    A = [   1.0  - 2.0 *qy*qy - 2.0 *qz*qz, 2.0 *qx*qy - 2.0 *qz*qw, ...
                2.0 *qx*qz + 2.0 *qy*qw, 0.0 ;
            2.0 *qx*qy + 2.0 *qz*qw, 1.0  - 2.0 *qx*qx - 2.0 *qz*qz, ...
                2.0 *qy*qz - 2.0 *qx*qw, 0.0 ;
            2.0 *qx*qz - 2.0 *qy*qw, 2.0 *qy*qz + 2.0 *qx*qw, ...
                1.0  - 2.0 *qx*qx - 2.0 *qy*qy, 0.0 ;
            0.0 , 0.0 , 0.0 , 1.0 ];   
end

function f()
%%
    syms q_w q_1 q_2 q_3;
    q = [ q_w, q_1, q_2, q_3 ];
    
    A = quatToMatrix(q);
    A
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