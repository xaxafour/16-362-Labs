

%counting numbers
x=zeros(1,1);
error=zeros(1,1);
i=1;
x(i)=i;
figure(2);

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
data=robot.laser.LatestMessage.Ranges;
data(data==0)=NaN;
positivemin=min(data);
    
[~,index]=find(data==positivemin);
hold on;
th=0;
plot(-positivemin*cos((90-th)*pi/180),positivemin*sin((90-th)*pi/180),'x');

error(i)=positivemin-1.2;

%check the error
disp(error(i));

sum=error(1);
while toc<=15
    robot.startLaser();
    pause(.2);
    data=robot.laser.LatestMessage.Ranges;
    data(data==0)=NaN;
    positivemin=min(data);
    i=i+1;
    x(i)=i;
    [row,index]=find(data==positivemin);
    
    %plot the point
    th=(index-1);
    hold on;
    plot(-positivemin*cos((90-th)*pi/180),positivemin*sin((90-th)*pi/180),'x');
    
    %PID control comes in
    error(i)=positivemin-1.2;
    
    % check the error
    disp(error(i));
    
    sum=sum+error(i);
    sub=error(i)-error(i-1);
    u=P*error(i)+I*sum+D*sub;
    
    s=u+1;
    
    if s>=1
      robot.sendVelocity(v-90*sin(th*pi/180)*w*v/s/pi,v+90*sin(th*pi/180)*w*v/s/pi);
    else
        robot.sendVelocity(-v+90*sin(th*pi/180)*w*v/s/pi,-v-90*sin(th*pi/180)*w*v/s/pi)
    end    
end
hold off
plot(x,error);
robot.close();
robot.shutdown();
    


