clear all
robot = neato('nano');
sp = 300; %mm

kp = .004;
ki = 0;
kd = 0.0;
errorIntegralMax = 5;



lasterror = 0;
l = robot.encoders.LatestMessage.Left;
r = robot.encoders.LatestMessage.Right;
mid0 = (l+r)/2;
mid(1) = (l+r)/2-mid0;
error(1) = sp;
derror(1) = 0;
ierror(1) = 0;
tic;

t(1) = toc;
i = 2;
%myPlot = plot(t,error);
%while toc < 4 && abs(error(i-1)) > .1; 
while toc < 4; 
    t(i) = toc;
    l = robot.encoders.LatestMessage.Left;
    r = robot.encoders.LatestMessage.Right;
    mid(i) = (l+r)/2 - mid0;
    error(i) = sp-mid(i);
    i
    error(i)
    i
    error(i-1)
    i
    (error(i) - error(i-1))
    derror(i) = (error(i) - error(i-1)) / (t(i) - t(i-1));
    ierror(i) = ierror(i-1) + error(i) * (t(i)-t(i-1));
    ierror(i) = scale(ierror(i),errorIntegralMax);
    
    upid(i) = kp * error(i) + ki * ierror(i) - kd* derror(i);
    upid(i) = scale(upid(i),0.2);
    robot.sendVelocity(upid(i),upid(i))
    pause(.1)
    
    l = robot.encoders.LatestMessage.Left;
    r = robot.encoders.LatestMessage.Right;
    mid(i) = (l+r)/2-mid0;
    error(i) = sp-mid(i);
    %set(myPlot, 'xdata', [get(myPlot,'xdata') t(i)], 'ydata', [get(myPlot,'ydata') error(i)]);
    i = i + 1;
end

robot.sendVelocity(0,0)
robot.close()
%robot.shutdown()
figure(1)
plot(t,derror,t,upid)
legend('derror','upid')
figure(2)
plot(t,error)