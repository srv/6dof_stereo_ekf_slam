function [resultVector timestamps statusRe] = imageRegistration( I1, I2, fCurrentLoop, pathLoop )
% This function implements the whoel update step for 3D EKF-SLAM
% IN:  I1 Left Stereo Image 
%      I2 Right Stereo Image 
%      fCurrentLoop contains all filenames of already observed images
%      pathLoop contains the path which is leading to the filenames
% OUT: resultVector contains all transformation from the current 3D
%         Position, observed by the sereo images to all found loop
%         closings. In terms of EKF this is Zk.
%      timestamps contains the timestampts of all found loop closings
%         in same order

% resultVector looks as follows: 
%      [transVec1, q1, transVec2, q2, ... , transVecn, qn]' 

% PARAM: numLoopClosings -> defines the maximum of LoopClosings (mainly for
%           testing. The more the better
%        numInliers      -> defines the minimum number of inliers
    numLoopClosings = 9;
    numInliersMin   = 17;

% In the case that no loop closing has been found we need to asign some
% values to the return parameters, otherwise MATLAB will strike
    resultVector = 0;
    timestamps   = 0;
    
% I. Find Correspondencies between Images
    [inlierOriginalLeft inlierOriginalRight descLeft status] = stereoMatching(I1, I2);
    
    if status ~= 0
        error('update:Stereo', 'Stereo Images does not fit to each other')
    end

%     counting the number of found loop closings
    count    = 0;
    statusRe = 0;
% II. Build Correspondencies between left Stereo Image and Loop-Closing
% Candidates
    for i = 1 : length(fCurrentLoop)
        I3 = imread([pathLoop '/' fCurrentLoop{i}]);
        [inlierPtsLeft, inlierPtsRight, inlierOriginalRightRed, status] = ...
            findLoopClosing(inlierOriginalLeft, inlierOriginalRight, descLeft, I3);
    
        if (status == 0 && length( inlierPtsLeft ) >= numInliersMin)
%             Show some Figures:
            stereoFig = figure(1);
            set(stereoFig,'name','Stereo Images','numbertitle','off');
            showMatchedFeatures(I1,I2,inlierPtsLeft,inlierPtsRight,'montage');
            
            loopclosingFig = figure(2);
            set(loopclosingFig,'name','Loop Closing','numbertitle','off');
            showMatchedFeatures(I1,I3,inlierPtsLeft,inlierPtsRight,'montage');
        
%       status: 0 = no error, 1 = input does not contain enough points, 
%               2 = Not enough inliers have been found.
% III. Perform backprojection
% III.a) Calculate 3D Points
            P3 = zeros(length( inlierPtsLeft ), 3);
            for m = 1:length( inlierPtsLeft )
                pTemp = calculate3DPoint(inlierPtsLeft(m).Location, ...
                                 inlierOriginalRightRed(m).Location);
                P3(m,1) = pTemp(1);
                P3(m,2) = pTemp(2);
                P3(m,3) = pTemp(3);
            end
%        Translate Feature to the internaly used format
            P2 = zeros(length(inlierPtsRight), 2);
            for n = 1:length(inlierPtsRight)
                ImgP = inlierPtsRight(n).Location;
                P2(n,1) = ImgP(1);
                P2(n,2) = ImgP(2);
            end
% III.b) Find object Pose (Transformation: Stereo -> Loop Closing)
            [tvec, q, rvec, numInliers] = objectPose3D2D(P3, P2);
                
% IV. Safe transformation to resultVector
%           If result uses less than n inliers discard it
            if( numInliers(1) >= numInliersMin )
%                 Otherwise add it to the resultVector
%                 Get timestamps from filename
                
                P = [663.847783169875 * 0.5, 0.0, 509.298866271973 * 0.5; ...
                     0.0, 663.847783169875 * 0.5 , 384.118202209473 * 0.5; ...
                     0.0, 0.0, 1.0 ];
                
                tVecP = P * tvec;
                angle = 2 * acos( q(1) );
                angle = angle * (180/pi);
                
                J = imtranslate(I1,[tVecP(2), tVecP(1), tVecP(3)]);
                J = imrotate(J, angle,'crop');
                
                lcMatchingFig = figure(3);
                set(lcMatchingFig,'name','Matching: Reference and LC','numbertitle','off');
                imshowpair(I3, J); 
%                 pause(0.05);
%                 [pitch roll yaw] = quat2angle(q, 'YXZ');
%                 qnew = angle2quat( pitch, roll, -yaw, 'YXZ' );
%                 q = qnew; 
                
% Add loop closing to the result vector               
% remove 'left_image' at the beginning and '.png' at the end
% result: timestamp
                tim  = fCurrentLoop{ i };
                timD = str2double( tim( 11:end-4 ) );
                if count == 0
                    statusRe     = 1;
                    resultVector = tvec;
                    resultVector = [ resultVector; q' ];
                    timestamps   = timD;
                else
                    resultVector = [ resultVector; tvec ];
                    resultVector = [ resultVector; q' ];
                    timestamps   = [ timestamps; timD ];
                end
                count = count + 1;
            end
        end
        
%         If we have reached the maximum of Loop Closings: return
        if (count == numLoopClosings)
            break;
        end
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