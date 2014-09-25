function [fNameLeft fNameRight pos status] = getStereoImageByTimestamp( timeStampOdo, ...
                                                        fLeft, fRight )
% This function return the stereo image pair corresponding to a certain
% timestamp. 
% INPUT:  timeStampOdo the timestamp provdided by odometry
%         fLeft set of all filenames of the left images
%         fRight set of all filenames of the right images
% OUTPUR: fNameLeft filename of the left image
%         fNameRight filename of the right image
%         status If no image pair has been found status will contain 0
%                otherwise 1
    
    pos = 0;
    status = 0;
    fNameLeft  = 'no image';
    fNameRight = 'no image';
    
    for i=1:length( fLeft )
        timeStampImage = fLeft{ i };
        timeStampImage = str2double( timeStampImage( 11:end-4 ) ); 
        if( abs( timeStampOdo - timeStampImage ) < 100000000)
            status = 1;
            fNameLeft  = fLeft{ i };
            fNameRight = fRight{ i };
            pos = i;
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