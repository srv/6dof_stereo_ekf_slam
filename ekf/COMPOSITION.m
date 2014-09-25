% Full 3D Frame Composition with six degress of freedom
%
% Do not forget to put the "util" subfolder to your path.
% 
% I.    System
% 
%   Xplus   = f ( x, y )
%   Xminus  = g ( x )
%
%   x       : state vector            
%   y       : measurement vector
%
%   Xplus   : Updated state vector 
%   Xminus  : Invers motion
%
% II.   Calculation
%   
%   Delivered with this Project as a PDF (subfolder doc/)
%
% III.  Plot result
%   
%   Full 3D Plot including robot movement and robot orientation
%
%
% IV.   Usage
%
%   Starting point is compMain.m. Alternatively and 100% working
%   is the approach using quaternions (all files endign with UQ
%   using quaternions). compMain.m reads a ROS-Bagfile given
%   from a visual odometry node. It takes the absolute state,
%   what means: rows 4, 5, 6, which are providing the position
%   and rows 7, 8, 9, 10 which are provifing the orientation as
%   a quaternion.
%   After this the relative motion is calculated using 
%   using relativeMotionFromAbsoluteMotion.m.
%   With the relative motion we are calculating with the composition-
%   function (comp.m) the absolute motion again.
%   If both motions fit perfectly we see that the composition- and
%   inversion-function are work well. 
%   For more information take a look at the subfolder doc/ for
%   the documentation and into the code.
%
%   Necessary files are:
%       1.  comp.m
%       2.  compMain.m
%       3.  plot_dir3.m
%       4.  invers.m
%       5.  quatInvers.m
%       6.  quatMult.m
%       7.  quatToMatrix.m
%       8.  relativeMotionFromAbsoluteMotion.m
%       9.  rosBagFileReader.m
%
%   All other files exist for better understanding (e.q. to get the 
%   solution as provided here).


% Copyright (c) 2014, Markus Solbach
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