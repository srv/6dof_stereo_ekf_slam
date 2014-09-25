function [inlierPtsLeft, inlierPtsRight, inlierOriginalRightUp, status] = findLoopClosing(inlierOriginalLeft, inlierOriginalRight, descLeft, I3)
% This function detects loop-closer.
% IN:  inlierOriginalLeft SURFPoints of left Image
%      inlierOriginalRight SURFPoints of right Image
%      descLeft SURF Discriptors of left Image
%      I3 Loop Closing candidate
% OUT: inlierPtsLeft if we have correspondencies between left and I3, this
%       will be set.
%      inlierPtsRight if we have correspondencies between left and I3, this
%       will be set.
%      inlierOriginalRightUp we need to update the inlier set of the right
%       stereo image as well.
%      status tells us if correspondencies have been found (0 = no error,
%       1 = input does not contain enough points, 2 = Not enough inliers
%       have been found)

    [desc3, SIFT3] = findFeature(I3);
    indexPairs = matchFeatures(descLeft, desc3, 'Prenormalized', true, 'MatchThreshold', 80);

    %  Update both sides of stereo images!
    inlierOriginalRight = inlierOriginalRight(indexPairs(:, 1));
    inlierOriginalLeft = inlierOriginalLeft(indexPairs(:, 1));
    matchedPoints2 = SIFT3(indexPairs(:, 2));

%     This function will fail if we have just one inlier
    if(length(inlierOriginalLeft) > 1)
        [Rt, inlierPtsLeft, inlierPtsRight, status] = ...
            estimateGeometricTransform(inlierOriginalLeft,matchedPoints2,'similarity');
        % Update index list
        index = zeros(length( inlierPtsLeft ), 1);
        for i = 1:length( inlierPtsLeft )
            InL = inlierPtsLeft(i).Location;
            for j = 1:length( inlierOriginalLeft )
                MaP = inlierOriginalLeft(j).Location;
                if InL == MaP
                    index(i) = j;
                    break;
                end
            end    
        end
    %  Index list of all discarded Feature of the left stereo image feature set
    %  Update right stereo image feature set
    inlierOriginalRightUp = inlierOriginalRight(index);
    else
        status = 2;
        inlierPtsLeft = 0;
        inlierPtsRight = 0;
        inlierOriginalRightUp = 0;
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