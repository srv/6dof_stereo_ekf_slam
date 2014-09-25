function plotEllipsoid( C, X )
% Util program to plot an ellipsoid
%   input:    X --> state [x, y, z, [q]] and C --> covariance ( 7 x 7 )
%   output:   grapical ellipsoid

% Samples
    N = 15;
    
% Take the uper-left 3x3 Matrix
% --> We are interested in the x, y, z covariances.
    cov = C(1:3, 1:3);
    
%   Resize-Factor
    rf = 1;

% Center
    c = [ X(1) X(2) X(3) ];
    
% Calculate semi-axis lengths
    [U S D] = svd( cov );
    
% Generate ellipsoid using matlab-function
    [x, y, z] = ellipsoid( c(1), c(2), c(3), sqrt(S(1,1)).*rf, ...
                            sqrt(S(2,2)).*rf, sqrt(S(3,3)).*rf, N );
    surf(x, y, z)
    axis equal
    
end


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