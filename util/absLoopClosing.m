function [LCH LCZ XREF] = absLoopClosing(X, hk, zk)
% This function calculates depending on the Loop Closings absolutes states
% to display them in the later in the trajectory to illustrate the LC
% IN  : X is the whole state vector
%       hk Loop Closing from the point of view of the state vector
%       zk Loop Closing from the point of view of the measurement
% OUT : LCH absolute states with respect to hk
%       LCZ absolute state with respect to zk
%       XREF absolute reference state

C = zeros(7,7);

% if ( LCH(1) == 0 )
%     allreadyFound = length(LCH)-7;
% else
%     allreadyFound = length(LCH)
% end
numLC = length( hk ) / 7;
% 
% XREF(allreadyFound+1:allreadyFound+7) = X(end-6:end);

XREF = X(end-6:end);

for i = 1:numLC
%     hkTemp = hk(i*7-6:i*7);
%     LCH(i*7-6+allreadyFound:i*7+allreadyFound) = composition(XREF, C, hkTemp, C);
%     
%     zkTemp = zk(i*7-6:i*7);
%     LCZ(i*7-6+allreadyFound:i*7+allreadyFound) = composition(XREF, C, zkTemp, C);

    hkTemp = hk(i*7-6:i*7);
    LCH(i*7-6:i*7) = composition(XREF, C, hkTemp, C);
    
    zkTemp = zk(i*7-6:i*7);
    LCZ(i*7-6:i*7) = composition(XREF, C, zkTemp, C);
    
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