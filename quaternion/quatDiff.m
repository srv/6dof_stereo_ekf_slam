function qDiff = quatDiff(q1, q2)
% Calculates the angular difference of two quaternions
% Remember quaternion representation is as follows: q = [ qw q2 q3 q4]

% 0. Re-Order quaternions (should be refactored at some time in the future)
%    Due to using matlab build-in functions (it has to be fast and correct)
    qw    = q1(1);
    q1(1) = q1(2);
    q1(2) = q1(3);
    q1(3) = q1(4);
    q1(4) = qw;
    
    qw    = q2(1);
    q2(1) = q2(2);
    q2(2) = q2(3);
    q2(3) = q2(4);
    q2(4) = qw;

% I. Get Euler angles of quaternion q1 and q2
    [pitchq1 rollq1 yawq1] = quat2angle(q1, 'YXZ')
    [pitchq2 rollq2 yawq2] = quat2angle(q2, 'YXZ')
       
% II. Get distance between each angle [ roll pitch yaw ]
    pitchDiff = pitchq1 - pitchq2;
    rollDiff  = rollq1 - rollq2;
    yawDiff   = yawq1 - yawq2;
        

% III. Get quaternion from euler angles    
    qDiff = angle2quat( pitchDiff, rollDiff, yawDiff, 'YXZ' );
    
% DEBUG: Calculate the angles from the result quaternion
    [pitch roll yaw] = quat2angle(qDiff, 'YXZ');
    [pitch roll yaw] * 180/pi
    
% IV. Re-Order
    qw       = qDiff(4);
    qDiff(4) = qDiff(3);
    qDiff(3) = qDiff(2);
    qDiff(2) = qDiff(1);
    qDiff(1) = qw;
    
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