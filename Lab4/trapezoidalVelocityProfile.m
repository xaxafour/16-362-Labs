function uref  = trapezoidalVelocityProfile( t , dist, sign)
    amax = .75;
    vmax = .25;
    tramp = vmax/amax;
    tf = abs(dist) / vmax + vmax/amax;
    if t < 0
        uref = 0;
    elseif t < tramp
        uref = sign *(amax*t);

    elseif t > tramp && t < tf-tramp
        uref = sign * vmax;

    elseif t > (tf - tramp) && t < tf
        uref = sign *(vmax -amax*(t-tf+tramp));
    else
        uref = 0;
    end
end