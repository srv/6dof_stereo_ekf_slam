
Ileft = imread('debug/left.png');
Iright = imread('debug/right.png');
Iloop = imread('debug/loop.png');

figure(3);
imshow(Iloop);

[inlierOriginalLeft inlierOriginalRight descLeft status] = stereoMatching(Ileft, Iright);

[inlierPtsLeft, inlierPtsRight, inlierOriginalRightRed, status] = ...
            findLoopClosing(inlierOriginalLeft, inlierOriginalRight, descLeft, Iloop);
        
figure(9); showMatchedFeatures(Ileft,Iloop,inlierPtsLeft,inlierPtsRight,'montage');
        
numInlier = length(inlierPtsLeft);

P3 = zeros(numInlier, 3);
   for m = 1:numInlier
        pTemp = calculate3DPoint(inlierPtsLeft(m).Location, ...
                                       inlierOriginalRightRed(m).Location);
        P3(m,1) = pTemp(1);
        P3(m,2) = pTemp(2);
        P3(m,3) = pTemp(3);
   end
   
P2 = zeros(length(inlierPtsRight), 2);
    for n = 1:length(inlierPtsRight)
        ImgP = inlierPtsRight(n).Location;
        P2(n,1) = ImgP(1);
        P2(n,2) = ImgP(2);
    end

[tvec, q, rvec, numInliers] = objectPose3D2D(P3, P2);

qGround = [ 0.0648681293 -0.0078227897 -0.1515113944 -0.986293682 ];

figure(3);
[pitchZ rollZ yawZ] = quat2angle(q, 'YXZ');
yawZ = yawZ * 180/pi;
J = imrotate(Ileft, yawZ,'bilinear');

K = [749.642742046463 * 0.5, 0.0, 539.67454188334 * 0.5; ...
           0.0, 718.738253774844 * 0.5, 410.819033898981 * 0.5; ...
           0.0, 0.0, 1.0
        ];
tVecP = K * tvec;
J = imtranslate(J,[-tVecP(1), tVecP(2), tVecP(3)]);
imshowpair(Iloop, J);
figure(5);
imshow(Iloop);
figure(4);
imshow(J);

[pitchGT rollGT yawGT] = quat2angle(q, 'YXZ');
yawGT = yawGT * 180/pi;
JGT = imrotate(Ileft, yawGT,'bilinear');
figure(2);
imshow(JGT);

x = [ tvec; q' ];