function norm = normState(in)
% This function computes the norm of a 7D state
    
    norm = sqrt( in(1) * in(1) + in(2) * in(2) + in(3) * in(3) + ...
                 in(4) * in(4) + in(5) * in(5) + in(6) * in(6) + ...
                 in(7) * in(7) );
end