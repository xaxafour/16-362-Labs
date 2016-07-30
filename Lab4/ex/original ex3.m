clear all
robot = neato('nano');
sp = .5; 
tic;
amax = .75;
vmax = .25;
tramp = vmax/amax;
tf = abs(sp) / vmax + vmax/amax;
pos(1) = 0;
time(1) = toc;
u(1) = 0;
i = 1;
if sp >= 0
    sign = 1;
else
    sign = -1;
end
%h= plot(time, u,'YDataSource', 'u', 'XDataSource', 'time');
%hold on
%h1= plot(time, pos,'YDataSource', 'pos', 'XDataSource', 'time');
%xlim([0.0 0.5]);
%ylim([0.0 0.5]);

while time(i) < tf; 
   i= i+ 1;
   time(i) = toc
   uref(i) = trapezoidalVelocityProfile(time(i),sp,sign);
   pos(i) = pos(i-1) + uref(i) * (time(i)-time(i-1));
   robot.sendVelocity(uref(i),uref(i))
   %set(h, 'Xdata',time(i),'YData',u(i));
   %set(h1, 'Xdata',time(i),'Ydata',pos(i));
   %drawnow;
   pause(0.01)
end

robot.sendVelocity(0,0)
robot.close()
%robot.shutdown()

plot(time,uref,time,pos)