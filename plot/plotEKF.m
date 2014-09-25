%% Plotting results
% It's old Code and probably it has to be change in the near future to
% get a more dynamic experience.

clf(trajectoriyFig);
hold on;

% Plot Covariance
% III. plot error ellipsoid
mean = [XOdom(end-6) XOdom(end-5) XOdom(end-4)];
% error_ellipse( Cnew(1:3,1:3), mean );
% plotEllipsoid( cov(1:3,1:3), mean )
xlabel('x');
ylabel('y');
zlabel('z');

% Plot pure odometry
[oX oY oZ] = stateVectorToXYZ(XOdom);
type = '-xk';
plot3(oX', oY', oZ', type, 'LineWidth',2);

% Plot updated state-vector
[cX, cY, cZ] = stateVectorToXYZ(X);
% [pitch, roll, yaw] = stateVectorToPitchRollYaw(X);
% trajectory_MFILE(cX, cY, cZ, pitch, roll, yaw, 800, 0.5);
type = '-diamondr';
plot3(cX', cY', cZ', type, 'LineWidth',2);


% Plot groundtruth
% Get groundtruth
gt = rosBagFileReader(2);
dX   = gt( :, 2 );
dY   = gt( :, 3 );
dZ   = gt( :, 4 );
dq1   = gt( :, 5 );
dq2   = gt( :, 6 );
dq3   = gt( :, 7 );
dqw   = gt( :, 8 );
    
q = [dqw, dq1, dq2, dq3];

% Add 90 degree
q = addAngleToQuaternion(q, 0, 0, pi/2);

[pitch, roll, yaw] = quat2angle(q, 'YXZ');

% trajectory_MFILE(dX, dY, dZ, pitch, roll, yaw, 800, 0);
[dX, dY, dZ] = rotatePositions(dX, dY, dZ, 0, 0, pi/2);
type = '-ob';
plot3(dX, dY, dZ, type, 'LineWidth',2);

% 
% if ( exist('LCH') )
%     numLC = length( LCH ) / 7;
%     for i = 1:numLC
%     % Plot Loop Closings correspondencies of the state vector (hk)
%         type = '-*c';
%         plot3([XREF(1) LCH(i*7-6)], [XREF(2) LCH(i*7-5)], [XREF(3) LCH(i*7-4)], type);
% 
%     % Plot Loop Closings given by the measurement (zk)    
%         type = '-squarem';
%         plot3([XREF(1) LCZ(i*7-6)], [XREF(2) LCZ(i*7-5)], [XREF(3) LCZ(i*7-4)], type);
%     end
% end

if ( LCH(1) ~= -1 )
% Plot Loop Closings correspondencies of the state vector (hk)
%         type = '-*c';
%         plot3([XREF(1) LCH(1)], [XREF(2) LCH(2)], [XREF(3) LCH(3)], type, 'LineWidth', 2);

% Plot Loop Closings given by the measurement (zk)    
        type = '-squareg';
%         plot3([XREF(1) LCZ(1)], [XREF(2) LCZ(2)], [XREF(3) LCZ(3)], type, 'LineWidth', 2);
end

if ( LCH(1) ~= -1 )
%     legend('Odometry', 'EKF-SLAM', 'Ground Truth', 'Loop Closing')
else
    legend('Odometry', 'EKF-SLAM', 'Ground Truth')
end
legend('Odometry', 'EKF-SLAM', 'Ground Truth')

LCH(1) = -1;

%title('comparison of localization')
grid on;
hold off;
%axis equal;
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