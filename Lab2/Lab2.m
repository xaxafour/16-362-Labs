clear;
rosshutdown;
robot = neato('pico');

%counting numbers
x=zeros(1,1);
error=zeros(1,1);
i=1;
x(i)=i;
figure(2);

distance = 1.3;
P=0.5;
I=0;
D=0.5;
w=0.03;
v=0.15;



%start to follow
grid on;
tic;
robot.startLaser();
pause(.2);
data = robot.laser.LatestMessage.Ranges;
data(data==0)=NaN;
positivemin=min(data);
    
[~,index]=find(data==positivemin);
hold on;
th=0;
plot(-positivemin*cos((90-th)*pi/180),positivemin*sin((90-th)*pi/180),'x');
xlim([-2, 2]);
ylim([-2, 2]);

error(i)=positivemin - distance;

%check the error
disp(error(i));

sum=error(1);
while toc<=60
    robot.startLaser();
    pause(.2);
    data=robot.laser.LatestMessage.Ranges;
    data(data==0)=NaN;
    [positivemin, index] = min(data);
    i=i+1;
    x(i)=i;
    
    %plot the point
    th=(index-1);
    hold on;
    plot(-positivemin*cos((90-th)*pi/180),positivemin*sin((90-th)*pi/180),'x');
    
    %PID control comes in
    error(i)=positivemin-0.8;
    
    % check the error
    disp(error(i));
    
    sum=sum+error(i);
    sub=error(i)-error(i-1);
    %u=P*error(i)+I*sum+D*sub;
    u = error(i);
    
    s=u+1;
    
    %go forward
    if s > distance * 1.1
      vl = scale(v-90*sin(th*pi/180)*w*v/s/pi);
      vr = scale(v+90*sin(th*pi/180)*w*v/s/pi);
      display(vl);
      display(vr);
      display(th);
      robot.sendVelocity(vl, vr);
    %go backward
    elseif s < distance * 0.9
      vr = scale(-v+90*sin(th*pi/180)*w*v/s/pi);
      vl = scale(-v-90*sin(th*pi/180)*w*v/s/pi);
      display(vl);
      display(vr);
      display(th);
      robot.sendVelocity(vl, vr);
    %stop
    else 
      robot.sendVelocity(0, 0);
    end
end
hold off
plot(x,error);
robot.close();
    


