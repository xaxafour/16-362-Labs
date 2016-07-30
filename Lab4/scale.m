function [out] = scale(in,max)
    if in >= 0
        sign = 1;
    else
        sign = -1;
    end
    
    if abs(in) > max
        out = sign*max;
    else
        out = in;
    end

end