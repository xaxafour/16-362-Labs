function newEncoderDataCallback2(obj,msg)
global sp kp ki kd l r mid t t0 l0 r0 i errorIntegralMax pos uref error derror ierror amax vmax tramp tf sign robot upid udelay posdelay usePID
if i==1
    l0=msg.Left;
    r0=msg.Right;
    t0=double(msg.Header.Stamp.Sec) + double(msg.Header.Stamp.Nsec) / 1000000000.0;
    t(1)=double(msg.Header.Stamp.Sec) + double(msg.Header.Stamp.Nsec) / 1000000000.0-t0;
    l(1)=double(msg.Left-l0)/1000.0;
    r(1)=double(msg.Right-r0)/1000.0;
    mid(1)=(l(1)+r(1))/2;
end
i = i+ 1;
t(i)=double(msg.Header.Stamp.Sec) + double(msg.Header.Stamp.Nsec) / 1000000000.0-t0;
l(i)=double(msg.Left-l0)/1000.0;
r(i)=double(msg.Right-r0)/1000.0;
mid(i)=(l(i)+r(i))/2;
%%forward
uref(i) = trapezoidalVelocityProfile(t(i),sp,sign);
pos(i) = pos(i-1) + uref(i) * (t(i)-t(i-1));
%udelay(i) = trapezoidalVelocityProfile(t(i)-0.8,sp,sign);
%posdelay(i) = posdelay(i-1) + udelay(i) * (t(i)-t(i-1));

%%feedback
% error(i)=pos(i)-mid(i);
% derror(i) = (error(i) - error(i-1)) / (t(i) - t(i-1));
% ierror(i) = ierror(i-1) + error(i) * (t(i)-t(i-1));
% ierror(i) = scale(ierror(i),errorIntegralMax);
% upid(i) = uref(i)+kp * error(i) + ki * ierror(i) + kd* derror(i); %%why minus
% upid(i) = scale(upid(i),0.29);


udelay(i) = trapezoidalVelocityProfile(t(i)-0.4,sp,sign);
posdelay(i) = posdelay(i-1) + udelay(i) * (t(i)-t(i-1));
error(i) = posdelay(i) - mid(i);
derror(i) = (error(i) - error(i-1)) / (t(i) - t(i-1));
ierror(i) = ierror(i-1) + error(i) * (t(i)-t(i-1));
ierror(i) = scale(ierror(i),errorIntegralMax);
if usePID
    upid(i) = uref(i)+kp * error(i) + ki * ierror(i) + kd* derror(i); %%why minus
else
    upid(i) = uref(i);
end
upid(i) = scale(upid(i),0.29);