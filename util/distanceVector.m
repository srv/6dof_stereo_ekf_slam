function [ diffVec maxV normV] = distanceVector(ekfXComp, gtXComp)
% This function calculates the distance of two vectors using the norm of
% each vectors
    diffVec = ekfXComp - gtXComp;

    maxV = max( diffVec );
    
    normV = norm( diffVec );

end