function Q = stereoCameraReproject()
%    Determine Reprojection Matrix (R)
%    From Springer Handbook of Robotics, p. 524
   
%          [ Fx 0  Cx -FxTx ]
%      K = [ 0  Fy Cy   0   ]
%          [ 0  0  1    0   ]

%          [fx'  0  cx' Fx*Tx]
%      P = [ 0  fy' cy' Ty]
%          [ 0   0   1   0]  

%          [ 1  0   0   -Cx ]
%      Q = [ 0  1   0   -Cy ]
%          [ 0  0   0    Fx ]
%          [ 0  0 -1/Tx  0  ]
%      parameters are from the left camera.

%          [ 1 0   0      -Cx      ]
%      Q = [ 0 1   0      -Cy      ]
%          [ 0 0   0       Fx      ]
%          [ 0 0 -1/Tx (Cx-Cx')/Tx ]

%     Cx = 553.634304046631;
%     Cy = 411.5126953125;
%     Fx = 670.44838436561;
    
%     Tx = ( -79.090353246545 * 0.5 ) / ( 670.44838436561 * 0.5 );
%     Cx = 553.634304046631 * 0.5;
%     Cy = 411.5126953125 * 0.5;
%     Fx = 670.44838436561 * 0.5;

    Tx = ( -79.090353246545 * 0.5 ) / ( 749.6427420 * 0.5 );
    Cx = 509.2988 * 0.5;
    Cy = 384.118202 * 0.5;
    Fx = 749.6427420 * 0.5;

    Q      = zeros(4,4);
    Q(1,1) = 1;
    Q(2,2) = 1;
    Q(1,4) = -Cx;
    Q(2,4) = -Cy;
    Q(3,4) = Fx;
    Q(4,3) = -1/Tx;
        
end

% %   Camera Parameter
%     intrinsic1 = [749.642742046463, 0.0, 539.67454188334; ...
%                     0.0, 718.738253774844, 410.819033898981; 0.0, 0.0, 1.0];
%     radial1     = [-0.305727818014552, 0.125105811097608, 0.0021235435545915]; 
%     tangential1 = [0.00101183009692414, 0.0];
%     
%     intrinsic2 = [747.473744648049, 0.0, 523.981339714942; ...
%                     0.0, 716.76909875026, 411.218247507688; 0.0, 0.0, 1.0];     
%     radial2     = [-0.312470781595577, 0.140416928438558, 0.00187045432179417]; 
%     tangential2 = [-0.000772438457736498, 0.0];  

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