timeStart = cputime;
    
trajectoriyFig = figure(4);
clf(trajectoriyFig);
set(trajectoriyFig,'name','Trajectory: Blue GT, Black Odometry, Red Updated','numbertitle','off');
title('comparison of localization')
set(trajectoriyFig, 'Position', [0 0 1600 900])
        
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% INITIALISATION
% clear;

predictionSetup;
updateSetup;

percent   = 0;
XOdom     = [0; 0; 0; 1; 0; 0; 0];
xTemp     = [0; 0; 0; 1; 0; 0; 0];
cPlcHldr  = zeros( 7, 7 );
tStateOdo = tMeasureOdo(1);
posOld    = -1;

counterUpdates = 0;
% LCH  = [0, 0, 0, 0, 0, 0, 0]; 
LCH(1) = -1;
% LCZ  = [0, 0, 0, 0, 0, 0, 0];
% XREF = [0, 0, 0, 0, 0, 0, 0];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% MAIN LOOP
for t = tt

%%     PREDICTION STEP
% xa1 is the first absolute Pose provided by libViso
% xa2 the second
    q1 = [aqw(t-1), aq1(t-1), aq2(t-1), aq3(t-1)];   
    q1 = quatNormal( q1 );    
    q1 = addAngleToQuaternion(q1, 0, 0, pi/2);
        
    q2 = [aqw(t), aq1(t), aq2(t), aq3(t)];
    q2 = quatNormal( q2 );    
    q2 = addAngleToQuaternion(q2, 0, 0, pi/2);
        
%     State Vectors (x, y, z, qw, qx, qy, qz)
    xa1  = [ aX(t-1), aY(t-1), aZ(t-1), q1(1), q1(2), q1(3), q1(4) ];
    xa2  = [ aX(t), aY(t), aZ(t), q2(1), q2(2), q2(3), q2(4) ];
    
    predictionCounter = length( xTemp ) / 7 + 1;
    xTempLast = xTemp( ((predictionCounter-2)*7)+1: (predictionCounter-1)*7 );
    
    xTempNew = prediction( xTempLast, cPlcHldr, xa1, xa2, cPlcHldr );
    xTemp = [xTemp;  xTempNew];
        
%     Only perform state augmentation and update every n Iterations.
    if ( mod(t, samplingRateSLAM) == 0 )
        
%         plotEKF;    
%         drawnow;
        
%%     STATE AUGMENTATION STEP
        updateCounter = length( X ) / 7 + 1;
        xTempLast = xTemp( ((predictionCounter-1)*7)+1: predictionCounter*7 );
        xLast     = X( ((updateCounter-2)*7)+1: (updateCounter-1)*7 );
        cLast     = C( (updateCounter-1)*7-6:(updateCounter-1)*7, (updateCounter-1)*7-6:(updateCounter-1)*7 );
%         cLast = eye(7,7);
        [Xnew, Cnew, Jac1, Jac2] = composition(xLast, cLast, xTempLast, CovRel, 1); 

%     Let the state-, covariance and timestamp-Vector grow
        X = [ X;  Xnew ];
        C = calcCov( C, Cnew, Jac1, Jac2 );  
                
%     safe timestamps of the odometry corresponding to each state
        tStateOdo = [ tStateOdo; tMeasureOdo(t) ];

%     safe the odometry for later debugging  
        xOdomLast     = XOdom( ((updateCounter-2)*7)+1: (updateCounter-1)*7 );
        [Xnew, Cnew, Jac1, Jac2] = composition(xOdomLast, cLast, xTempLast, CovRel, 1); 
        XOdom = [XOdom, Xnew];

%     Reset temporal state variable of the prediction step
        xTemp = [0; 0; 0; 1; 0; 0; 0];
        
%%      UPDATE STEP
%     Try to find Loop closing candidate with a certain sampling rate
    if mod(t,loopSample) == 0
        
%     Load Stereo Images from Database 
%       ( --> corresponding to the current timestamp of the odometry)
        [fNameLeft, fNameRight, pos, status]= getStereoImageByTimestamp(...
                                                    tMeasureOdo(t), ...
                                                    fLeft, fRight);
                                   
%     safe fNameLeft as already observed image in a new vector
        fLoop{ end+1 } =  fNameLeft;
        
        if ( ( status == 0 ) || ( updateCounter <= imageDiscard ) )
%           if status == 0 no corresponding stereo image pair has been
%           found: skip this
        else
% %           if status == 1 corresponding stereo image pair has been
% %           found: look for loop closing
%             ILeft  = imread([pathLeft '/' fNameLeft]);  
%             IRight = imread([pathRight '/' fNameRight]); 
%                     
% %           Pass already observed Images to update function (discard the
% %           last n (--> pos - n) )
%             fCurrentLoop = fLoop(1:end-imageDiscard);
%             
%             status = 0;
% 
%             [zk timestampsLC status] = imageRegistration( ILeft, IRight, ...
%                                                   fCurrentLoop, pathLeft );

%             Test the filter with correct image registration. 
            [zk timestampsLC status pos] = testImageRegistration( tMeasureOdo(t) );
            
            if ( posOld == pos )
                status = 0;
            else
                posOld = pos;
            end
            
            if( status == 1 )
%             If status is equal to 1 we have at least one loop closing
%             Don't forget to safe the timestamp of the reference Image
%             (left image) at the end of the timestamp vector
                timeRef = str2double( fNameLeft( 11:end-4 ) );
                timestampsLC = [ timestampsLC; timeRef ];

%             Applying the Kalman-Equations

%             Calculate h1 - hn: these are the realtive motions of states
%             taken from the state-vector (state estimations) corresponding
%             to the detected loop closing. In terms of EKF this is hk. the
%             parameter zk is important to update it inside this function.
%             In case we do not have to every loop closing a corresponding
%             odometry or vice versa.
                [hk H zk numLC] = calculateHhk( X, tStateOdo, ...
                                                    timestampsLC, zk );  
                                                             
%             If number of loop closings is equal to 0 no hk had been 
%             calculated so cancel all update procedures
              if ( numLC ~= 0 )
%             perform UPDATE for all found loop closings

%             DEBUGGING ( to show loopclosings )
                [LCH LCZ XREF] = absLoopClosing(X, hk, zk);
%             I.   Innovation: yk = zk - hk
                  yk = innovation(zk, hk);

%             II.  Innovation covariance: Sk = H * C * H^T + Rk 
%                  Build covariance Rk depending on #Loopclosings
                  Rk = buildRk( CovMeas, numLC );  
                  Sk = H * C * H' + Rk;

%             III. Kalman gain: K = C * H^T * Sk^-1
                  K  = (C * H') / Sk;
                  
%             IV.  Update state estimate: X = X + K*yk

                  plotEKF;
                  drawnow;
                 upda = 1;
                 if (upda == 1)
%                   plotEKF;    
%                   drawnow;
%                   frame = getframe(trajectoriyFig);
%                   writeVideo(writerObj, frame);
     
                  X  = X + K * yk;
                  
%                   d = max(max(K))
                  trajectoryError(X, XOdom, tStateOdo);
                                    
%             V.   update covariance estimate: C = ( 1-K*H ) * C
                  prodKH = K*H;
                  C  = ( eye( size( prodKH ) ) - prodKH ) * C;
                  counterUpdates = counterUpdates + 1
                 end
              end
              
%             [DEBUG] Called to see states during runtime 
%               plotEKF;
%               drawnow;
             end
          end
       end 
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% PLOT EVERYTHING
plotEKF;
drawnow;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% SHUTTING DOWN MATLAB
% clearMATLAB;

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