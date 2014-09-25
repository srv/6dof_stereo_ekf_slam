function [zk timestampsLC status posRef] = testImageRegistration(refTime)

%     define output
      zk = 0;
      timestampsLC = 0;
      status = 0;
      posRef = 0;

%     load precalculated loop closings
      data     = double( dlmread( 'bag/ROS/loop_closures_wo_errors.txt', ',' ) );
%       data     = double( dlmread( 'bag/ROS/loop_closures.txt', ',' ) );
        
%     find loop closing with refTime (2. Timestamp) as reference 
      posRef = max(find( abs(data(:,1) - refTime) < 100000000 ));

      if( posRef ~= 0)
%     save loopTime (2. Timestamp) to timestampsLC
        timestampsLC = data(posRef,2);

%     set zk as a vector of [x y z qw q1 q2 q3] - vectors
        zk(1) = data(posRef, 3);
        zk(2) = data(posRef, 4);
        zk(3) = data(posRef, 5);
        zk(4) = data(posRef, 9);
        zk(5) = data(posRef, 6);
        zk(6) = data(posRef, 7);
        zk(7) = data(posRef, 8);
        
%       TESTING:
        q = zk(4:7);
        t = zk(1:3);
        q = addAngleToQuaternion(q, 0, 0, pi/2);
%         zk(4:7) = q;
        
        R = angle2dcm( 0, 0, pi/2, 'YXZ' );
        t = R * t';    
        zk(1:3) = t(1:3);
        
        zk = zk';
        
        status = 1;
                
        posRef
      end
end