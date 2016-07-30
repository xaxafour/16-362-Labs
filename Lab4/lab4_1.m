
global sp kp ki kd l r mid t t0 l0 r0 i errorIntegralMax pos uref error derror ierror amax vmax tramp tf sign robot upid udelay posdelay usePID
usePID = true;
sp = 1;
kp = 1.5;
ki = 0.5;
kd = 0.1;
l=zeros(1,1);
r=zeros(1,1);
mid=zeros(1,1);
pos=zeros(1,1);
posdelay=zeros(1,1);
udelay=zeros(1,1);
t=zeros(1,1);
%t0 = double(robot.encoders.LatestMessage.Header.Stamp.Sec) + double(robot.encoders.LatestMessage.Header.Stamp.Nsec) / 1000000000.0;
%l0 = robot.encoders.LatestMessage.Left/1000.0;
%r0 = robot.encoders.LatestMessage.Right/1000.0;
i=1;
errorIntegralMax = 5;
pos(1) = 0;
uref(1) = 0;
upid(1)=0;
error(1)=0;
derror(1)=0;
ierror(1)=0;
posdelay(1) = 0;
udelay(1) = 0;
amax = .75;
vmax = .25;
tramp = vmax/amax;
tf = abs(sp) / vmax + vmax/amax;
if sp >= 0
    sign = 1;
else
    sign = -1;
end
%robot.encoders.NewMessageFcn = @newEncoderDataCallback2;
if usePID
    stopTime = tf+0.4+1;
else
    stopTime = tf+0.4;
end

figure(1);
myPlot1 = plot(t, posdelay);
hold on;
myPlot2 = plot(t, mid);
xlim([0 6]);
ylim([0 1.1]);
xlabel('time(s)');
ylabel('position(m)');
legend('reference', 'encoder');

figure(2);
myPlot3 = plot(t, error);
xlim([0 6]);
ylim([-0.1 0.1]);
xlabel('time(s)');
ylabel('error(m)');

loops = 0;
while t(i) < stopTime
    if loops < 5
        robot.sendVelocity(0, 0);
    elseif loops == 5
        robot.encoders.NewMessageFcn = @newEncoderDataCallback2;
    else
        robot.sendVelocity(upid(i),upid(i));
        %set(myPlot1,'Xdata',t(i),'Ydata',posdelay(i));
        %set(myPlot2,'Xdata',t(i),'Ydata',mid(i));
        %set(myPlot3,'Xdata',t(i),'Ydata',error(i));
        %drawnow;
        set(myPlot1, 'xdata', [get(myPlot1, 'xdata') t(i)], 'ydata', [get(myPlot1, 'ydata') posdelay(i)]);
        set(myPlot2, 'xdata', [get(myPlot2, 'xdata') t(i)], 'ydata', [get(myPlot2, 'ydata') mid(i)]);
        set(myPlot3, 'xdata', [get(myPlot3, 'xdata') t(i)], 'ydata', [get(myPlot3, 'ydata') error(i)]);
        drawnow;
    end
    
    loops = loops+1;
    pause(.01);
end
robot.sendVelocity(0,0);
robot.close();
%figure(1);
%plot(t, pos, t, posdelay, t, mid);
%legend('Pos', 'Posdelay', 'Mid');
%figure(2);
%plot(t,error);

    
