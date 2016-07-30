function uref  = trapezoidalVelocityProfile( t   , dist, sign)
    amax = .75;
    vmax = .25;
    tramp = vmax/amax;
    tf = abs(dist) / vmax + vmax/amax;
    if t < tramp
        uref = sign *(amax*t);
        1
        t
    elseif t > tramp && t < tf-tramp
        uref = sign * vmax;
        2
        t
    elseif t > (tf - tramp) && t < tf
        uref = sign *(vmax -amax*(t-tf+tramp));
        3
        t
    else
        uref = 0;
    end
end