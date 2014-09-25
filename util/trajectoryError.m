%% Trajectory Error between GT - Odometry and GT - EKF-Result
function [errorOdom, errorEKF, diffEKF_Roll, diffEKF_Pitch, diffEKF_Yaw ...
          diffOdom_Roll, diffOdom_Pitch, diffOdom_Yaw] = trajectoryError(X, XOdom, tStateOdo, o, type, timeTotal)
% This function calculates the error between GT - Odometry and GT - EKF
% Therefore it calculates the distance to a fixed reference point. The
% distances are calculated always with correspondend points. Let ID a
% unique number to identify the same pose in all state-vectors namely GT,
% Odom and X (EKF) and IDref the overall reference ID:
% diffGT   = GT(ID)   - GT(ID)
% diffOdom = Odom(ID) - Odom(IDref)
% diffEKF  = EKF(ID) - EKF(IDref)
%   where for example GT(ID) is the norm of the 3D state (X,Y,Z)
% errorOdom = sum(sqrt(diffOdom - diffGT)^2)
% errorEKF  = sum(sqrt(diffEKF - diffGT)^2)
% INPUT  : X is the EKF updated state-vector
%          XOdom is the pure Odometry (dead reckoning)
%          tStateOdo are all state-timestamps up to time
%          o is the current loop iteration
%          type which type of evaluation
% OUTPUT : errorOdom is the error of the Odometry
%          errorEKF is the error of the EKF-Approach
%          ANGLES the error of roll, pitch and yaw for EKF and Odom

% Variable for travelled meters
traveled = 0;

% Get groundtruth
gt = rosBagFileReader(2);
dT   = gt( :, 1 );
dX   = gt( :, 2 );
dY   = gt( :, 3 );
dZ   = gt( :, 4 );
dq1   = gt( :, 5 );
dq2   = gt( :, 6 );
dq3   = gt( :, 7 );
dqw   = gt( :, 8 );

% Find reference Indeces
tsGTIndexA   = 0;
tsTrajIndexA = 0;
for i = 1:length(dT)
   tsGT   = dT(i);
   tsTrajIndexA = min(find( abs(tStateOdo - tsGT) < 10000));
   if(tsTrajIndexA ~= 0)
      tsGTIndexA = i;
      break;
   end
end

% Find reference States
GTA   = [ dX(tsGTIndexA); dY(tsGTIndexA); dZ(tsGTIndexA) ];
qGTA = [ dqw(tsGTIndexA); dq1(tsGTIndexA); dq2(tsGTIndexA); dq3(tsGTIndexA) ];
[pitchGTA, rollGTA, yawGTA] = quat2angle(qGTA', 'YXZ');

EKFA  = [ X(tsTrajIndexA*7-6); X(tsGTIndexA*7-5); X(tsGTIndexA*7-4) ];
qEKFA = [ X(tsTrajIndexA*7-3); X(tsGTIndexA*7-2); X(tsGTIndexA*7-1); X(tsGTIndexA*7) ];
[pitchEKFA, rollEKFA, yawEKFA] = quat2angle(qEKFA', 'YXZ');

OdomA = [ XOdom(tsTrajIndexA*7-6); XOdom(tsGTIndexA*7-5); XOdom(tsGTIndexA*7-4) ];
qOdomA = [ XOdom(tsTrajIndexA*7-3); XOdom(tsGTIndexA*7-2); XOdom(tsGTIndexA*7-1); XOdom(tsGTIndexA*7) ];
[pitchOdomA, rollOdomA, yawOdomA] = quat2angle(qOdomA', 'YXZ');

% Prepare error vectors. Due to this initialisation they are forced to be
% column-vetor
diffEKF_GTU  = [ 0; 0 ];
diffEKF_Roll = [ 0; 0 ];
diffEKF_Pitch = [ 0; 0 ];
diffEKF_Yaw = [ 0; 0 ];

diffOdom_GTU = [ 0; 0 ];
diffOdom_Roll = [ 0; 0 ];
diffOdom_Pitch = [ 0; 0 ];
diffOdom_Yaw = [ 0; 0 ];

% Find reference in the following trajectory between GT and EKF
tsGTIndexB   = 0;
tsTrajIndexB = 0;
counter = 0;
for j = tsGTIndexA:length(dT)
    tsGTIndexB = dT(j);    
    tsTrajIndexB = min(find( abs(tStateOdo - tsGTIndexB) < 100000));
%     If we found a corresponding timestamp let's compute the error
    if(tsTrajIndexB ~= 0)
        counter = counter +1;

%         Save traveled meters
        if(counter == 1)
            traveled = traveled + euclidDistance( [GTA(1); GTA(2); GTA(3)], [dX(j); dY(j); dZ(j)] );
        else
            traveled = traveled + euclidDistance( [GTB(1); GTB(2); GTB(3)], [dX(j); dY(j); dZ(j)] );
        end
%         I.   Calculate the difference between GT  Point A and B
        GTB = [ dX(j); dY(j); dZ(j) ]; 
        [ pitchGTB, rollGTB, yawGTB ] = quat2angle( [dqw(j); dq1(j); dq2(j); dq3(j)]' , 'YXZ' );
        [ diffVecGT maxVGT normVGT ] = distanceVector(GTA, GTB);  
        
        
%         II.  Calculate the difference between EKF Corresponding Point A and B
        EKFB = [ X(tsTrajIndexB*7-6); X(tsTrajIndexB*7-5); X(tsTrajIndexB*7-4) ];
        [ pitchEKFB, rollEKFB, yawEKFB ] = quat2angle( [X(tsTrajIndexB*7-3); X(tsTrajIndexB*7-2); X(tsTrajIndexB*7-1);X(tsTrajIndexB*7)]' , 'YXZ' );
        [ diffVecEKF maxVEKF normVEKF ] = distanceVector(EKFA, EKFB);

%         III.  Calculate the difference between Odom Corresponding Point A and B
        OdomB = [ XOdom(tsTrajIndexB*7-6); XOdom(tsTrajIndexB*7-5); XOdom(tsTrajIndexB*7-4) ];
        [ pitchOdomB, rollOdomB, yawOdomB ] = quat2angle( [XOdom(tsTrajIndexB*7-3); XOdom(tsTrajIndexB*7-2); XOdom(tsTrajIndexB*7-1);XOdom(tsTrajIndexB*7)]' , 'YXZ' );
        [ diffVecOdom maxVOdom normVOdom ] = distanceVector(OdomA, OdomB);

%         IV.   Differences bettween each and GT
        currentDiff   = normVGT - normVEKF;
        diffEKF_GTU(counter) = sqrt( currentDiff * currentDiff );
        
        currentDiff   = normVGT - normVOdom;
        diffOdom_GTU(counter) = sqrt( currentDiff * currentDiff );
        
        currentDiff   = (pitchGTB*180/pi) - (pitchEKFB*180/pi);
        diffEKF_Pitch(counter) = sqrt( currentDiff * currentDiff );
        currentDiff   = (rollGTB*180/pi) - (rollEKFB*180/pi);
        diffEKF_Roll(counter) = sqrt( currentDiff * currentDiff );
        currentDiff   = (yawGTB*180/pi) - (yawEKFB*180/pi);
        diffEKF_Yaw(counter) = sqrt( currentDiff * currentDiff );
        
        currentDiff   = (pitchGTB*180/pi) - (pitchOdomB*180/pi);
        diffOdom_Pitch(counter) = sqrt( currentDiff * currentDiff );
        currentDiff   = (rollGTB*180/pi) - (rollOdomB*180/pi);
        diffOdom_Roll(counter) = sqrt( currentDiff * currentDiff );
        currentDiff   = (yawGTB*180/pi) - (yawOdomB*180/pi);
        diffOdom_Yaw(counter) = sqrt( currentDiff * currentDiff );
        
    end
end

% Build the sum of the error vectors and divide by traveled to get the
% error per traveled meters ratio
errorOdom = sum(diffOdom_GTU) / traveled
errorEKF  = sum(diffEKF_GTU) / traveled

errorOdomPitch = sum(diffOdom_Pitch) / traveled;
errorOdomRoll = sum(diffOdom_Roll) / traveled;
errorOdomYaw = sum(diffOdom_Yaw) / traveled;

errorEKFPitch = sum(diffEKF_Pitch) / traveled;
errorEKFRoll = sum(diffEKF_Roll) / traveled;
errorEKFYaw = sum(diffEKF_Yaw) / traveled;

traveled

if nargin > 3
    output = traveled;
    output = [output; errorOdom];
    output = [output; errorEKF];
    output = [output; errorOdomPitch];
    output = [output; errorOdomRoll];
    output = [output; errorOdomYaw];
    output = [output; errorEKFPitch];
    output = [output; errorEKFRoll];
    output = [output; errorEKFYaw];
    output = [output; timeTotal];

    no = int2str(o);
    file = 'out/';
    file = strcat(file, type);
    file = strcat(file, '-');
    file = strcat(file, no);
    file = strcat(file, '.txt');
    dlmwrite(file, output,'precision','%.6f', 'delimiter','\n');
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