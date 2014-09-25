function [h1, h2] = plot_dir3 (vX, vY, vZ, color)
%Plotting x, y, z variables with direction indicating vector to the next element.
%Example
%   vX = linspace(0,2*pi, 10)';
%   vY = sin (vX);
%   vZ = cos (vX);
%   plot_dir3(vX, vY, vZ);

% row / column vector check
if ( isrow(vX) )
    vX = vX';
end

if ( isrow(vY) )
    vY = vY';
end

if ( isrow(vZ) )
    vZ = vZ';
end

rMag = 0.5;

% Length of vector
lenTime = length(vX);

% Indices of tails of arrows
vSelect0 = 1:(lenTime-1);
% Indices of tails of arrows
vSelect1 = vSelect0 + 1;

% X coordinates of tails of arrows
vXQ0 = vX(vSelect0, 1);
% Y coordinates of tails of arrows
vYQ0 = vY(vSelect0, 1);
% X coordinates of tails of arrows
vZQ0 = vZ(vSelect0, 1);

% X coordinates of heads of arrows
vXQ1 = vX(vSelect1, 1);
% Y coordinates of heads of arrows
vYQ1 = vY(vSelect1, 1);
% Z coordinates of heads of arrows
vZQ1 = vZ(vSelect1, 1);

% vector difference between heads & tails
vPx = (vXQ1 - vXQ0) * rMag;
vPy = (vYQ1 - vYQ0) * rMag;
vPz = (vZQ1 - vZQ0) * rMag;

if  color == 'r'
    % make plot 
    h1 = plot3 (vX, vY, vZ, 'r--');
    % add arrows 
    h2 = quiver3 (vXQ0, vYQ0, vZQ0, vPx, vPy, vPz, 0, color); grid on; 
elseif color == 'b'
    % make plot 
    h1 = plot3 (vX, vY, vZ, 'b-');
    % add arrows 
    h2 = quiver3 (vXQ0, vYQ0, vZQ0, vPx, vPy, vPz, 0, color); grid on; 
else
    % make plot 
    h1 = plot3 (vX, vY, vZ, 'r-');
    % add arrows 
    h2 = quiver3 (vXQ0, vYQ0, vZQ0, vPx, vPy, vPz, 0, 'b'); grid on; 
    legend(' robot movement ',' robot orientation ');
end

axis equal

text(vX(1),vY(1), vZ(1),['start'],...
     'FontSize',8)
 
text(vX(lenTime),vY(lenTime), vZ(lenTime),['end'],...
     'FontSize',8)

ii = 1:1:lenTime;
for i = ii
%     if mod(i,100) == 0
%         text(vX(i),vY(i), vZ(i),['x = ',num2str(vX(i)), ', ', ...
%             num2str(vY(i)), ', ', num2str(vZ(i))],...
%             'FontSize',8)
%     end
%     if mod(i,105) == 0
%         text(vX(i),vY(i), vZ(i),['t = ',num2str(i)],...
%             'FontSize',6)
%     end
end

%      text(vX(3),vY(3), vZ(3),['t = ',num2str(3)],...
%          'FontSize',8)
%      
%      text(vX(11),vY(11), vZ(11),['t = ',num2str(11)],...
%          'FontSize',8)


% Copyright (c) 2010, Kangwon Lee
% Update: Markus Solbach [17.04.2014]
%       - legend
%       - invariance to row / column vector
%       - vector annotation (18.04.2014)
%       - plotting of more than one trajectorie (28.04.2014)
%       - supports new parameter for color (28.04.2014)

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