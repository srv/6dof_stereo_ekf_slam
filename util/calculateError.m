%% Calculating Error between GT - Odometry and GT - EKF-Result

%  Build new state Vector from Ground Truth Data
% Get groundtruth
gt = rosBagFileReader(2);
dX   = gt( :, 2 );
dY   = gt( :, 3 );
dZ   = gt( :, 4 );
dq1   = gt( :, 5 );
dq2   = gt( :, 6 );
dq3   = gt( :, 7 );
dqw   = gt( :, 8 );
XGT   = [0; 0];
for i = 1:length(dX)
    XGT(i*7-6) = gt( i, 2 );
    XGT(i*7-5) = gt( i, 3 );
    XGT(i*7-4) = gt( i, 4 );
    XGT(i*7-3) = gt( i, 8 );
    XGT(i*7-2) = gt( i, 5 );
    XGT(i*7-1) = gt( i, 6 );
    XGT(i*7)   = gt( i, 7 );
end

% Taking each Element (sate) of the EKF-Result and find the closest Element
% (state) in XGT
errorOdom = [0;0];
errorEKF  = [0;0];
smallestIndex = 0;
maxOld = 1000000;

for j = 1:(length(X)/7)
    ekfXComp = X(j*7-6:j*7-5);
    j
    maxOld = 1000000;
%     find the best GT data
    for k = 1:(length(XGT)/7)
        gtXComp = XGT(k*7-6:k*7-5);
        [ diff, max ] = distanceVector(ekfXComp, gtXComp);
        if (max < maxOld)
            maxOld = max;
            smallestIndex = k;
        end
    end
%     after finding it --> calculate the error between 
%     GT - Odometry and GT - EKF-Result
    gtXComp = XGT(smallestIndex*7-6:smallestIndex*7-5);
    odomXComp = XOdom(j*7-6:j*7-5); 
    [ diffVOdom, maxVOdom, normVOdom ] = distanceVector(odomXComp', gtXComp);
    errorOdom(j) = normVOdom;     
    [ diffVEKF, maxVEKF, normVEKF ] = distanceVector(ekfXComp, gtXComp);
    errorEKF(j)  = normVEKF;
end

% Calculate Average Error
numSamples = length(X)/7;

sumOdom = sum(errorOdom)
sumEKF  = sum(errorEKF)

averageErrorOdom = sumOdom / numSamples;
averageErrorEKF  = sumEKF / numSamples;

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