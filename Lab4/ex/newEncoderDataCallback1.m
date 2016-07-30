function newEncoderDataCallback1(obj,msg)
global i l time t0 l0 pos uref sp sign
i=i+1;
if i < 2
    i = 2;
end
if i == 2
    l0 = msg.Left;
    t0 = double(msg.Header.Stamp.Sec) + double(msg.Header.Stamp.Nsec) / 1000000000.0;
end
l(i) = msg.Left-l0;
time(i) = double(msg.Header.Stamp.Sec) + double(msg.Header.Stamp.Nsec) / 1000000000.0 - t0;
uref(i) = trapezoidalVelocityProfile(time(i-1),sp,sign);
pos(i) = pos(i-1) + uref(i) * (time(i)-time(i-1));
end
