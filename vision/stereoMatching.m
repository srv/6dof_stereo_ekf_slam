function [inlierPtsLeft inlierPtsRight descLeft status] = stereoMatching(I1, I2)
%   find correspondencies between images
%   IN :    I1, I2 (both Images of a stereo image system)
%   OUT:    inlierPtsLeft inliers of the left image in corresponding order
%           as SURFPoints
%           inlierPtsRight inliers of the right image in corresponding order as
%           SURFPoints
%           descLeft descriptors of SURFPoints of the left image
%           satus tells us if correspondencies have been found (0 = no error,
%               1 = input does not contain enough points, 2 = Not enough inliers
%               have been found)

%     I. Find Feature (SURF)
    [f1, vpts1] = findFeature(I1);
    [f2, vpts2] = findFeature(I2);
    
%     II. Find Correspondencies (with outlier elimination)

    [inlierPtsLeft, inlierPtsRight, Rt, status, descLeft] = findCorrespondenciesIndex(f1, vpts1, f2, vpts2);
    
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