clear
global i l time t0 l0 uref pos sp sign
robot = neato('yotta');
sp = .5; 
amax = .75;
vmax = .25;
tramp = vmax/amax;
tf = abs(sp) / vmax + vmax/amax;
pos(1) = 0;
uref(1) = 0;
time(1) = 0;
%l0 = robot.encoders.LatestMessage.Left / 1000;
%l(1) = robot.encoders.LatestMessage.Left / 1000 - l0;
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

loops = 0;

while time(i) < tf; 
   %uref(i) = trapezoidalVelocityProfile(time(i-1),sp,sign);
   %if i ~= 1
   %pos(i) = pos(i-1) + uref(i) * (time(i)-time(i-1));
   %end
   %vel(i) = (l(i) - l(i-1)) / (time(i)-time(i-1));
   %robot.sendVelocity(scale(uref(i)), scale(uref(i)));
   %set(h, 'Xdata',time(i),'YData',u(i));
   %set(h1, 'Xdata',time(i),'Ydata',pos(i));
   %drawnow;
   
   if loops < 5
       robot.sendVelocity(0, 0);
   elseif loops == 5
       robot.encoders.NewMessageFcn = @newEncoderDataCallback1;
   else
       robot.sendVelocity(scale(uref(i)), scale(uref(i)));
   end
   
   loops = loops+1;
   pause(0.01);
end

robot.sendVelocity(0,0)
robot.close();

figure(1);
plot(time,pos,time,l);
figure(2);
plot(time,uref);