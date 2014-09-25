function [hk H zk numLC] = calculateHhk( X, tStateOdo, timestampsLC, zk )
% This function calculates the relative estimated motion from with respect
% to the observed landmarks.
% INPUT  : X state vector containing all current states 
%          tStateOdo vector with corresponding (to state vector) timestamps 
%          timestampsLC vector with the timestamps of the loop closings
% OUTPUT : hk is the akkumulated estimation
%          H is the H Matrix as known from EKF
%          numLC tells us how many elements finally have been used for the
%           loop closing. In case not to each Image exists an Odometry or
%           vice versa.

    numLC = 0;
%     because in this case the covariance doesn't matter we just set it to
%     a zero matrix
    cov  = zeros( 7, 7 );

%     create H Matrix MxN, where M is the sice of loop closings times 7 and
%     N the number of states, because each entry in H will be 7x7:
%     this is the size of each Jacobians, because every state consist of 7
%     values
%     H = zeros( (length(timestampsLC) - 1)*7, length(X) );
    
%     create hk filled with zeros for easier access afterwards
%     hk = zeros( (length(timestampsLC)-1)*7,1);
%     load absolute motions from state vector
%     load reference state
    timeRef = timestampsLC( end );
    
    posRef = find( abs( tStateOdo - timeRef ) < 100000000 );
    
    if( length( posRef ) > 1)
        posRef = posRef( length( posRef ) );
    end
    
    xRef = X( (posRef(1)*7-6):(posRef(1)*7) );
    
%     safe the number of discarded elements
    disc = 0;
    pos  = 0;
    
    for i=1:(length(timestampsLC) - 1)
        pos = max( find( abs( tStateOdo - timestampsLC( i ) ) < 100000000) );
        
        if( pos ~= 0 )
            numLC = numLC + 1;
            x1  = X( (pos(1)*7-6):(pos(1)*7) );
        
%           calculate relative motion. 
            [ h cov Jac1 Jac2 ] = relativeMotionFromAbsoluteMotionUQ(xRef, cov, ...
                                                             x1, cov);
        
%           push the both Jacobians to H-Matrix
            H( numLC*7-6:numLC*7 , (pos(1)*7-6):(pos(1)*7))       = Jac2;
            H( numLC*7-6:numLC*7 , (posRef(1)*7-6):(posRef(1)*7)) = Jac1;
        
%           push relative motion to hk vector
            hk( numLC*7-6:numLC*7 ) = h;
        else
%           If no odometry could be found corresponding to the loop closing
%           we need to discard this loop closing from the zk vector.
            if( (i-disc) == 1 ) 
                zk = zk( (i-disc)*7+1:end );
                disc = disc + 1;
            else
                zk = zk([1:(i-1-disc)*7, (i-disc)*7+1:end]);
                disc = disc + 1;
            end
        end
    end
    
    if( numLC ~= 0 )
        hk = hk';
    else
        hk = 0;
        H  = 0;
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