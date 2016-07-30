function newEncoderDataCallback(obj,msg)
format long
global neatoEncoderFrame
global l r
global w
% global robot
% global time
global t;
global i;
global j;
global dr;
global dl;
global sum;
global theta;
global dm;
global x;
global y;
global sumx sumy
% global time0
% global oldTime
%neatoEncoderDataTimestamp = event.data.header.stamp.secs + (event.data.header.stamp.nsecs/1000000000.0);

neatoEncoderFrame = neatoEncoderFrame + 1;
% oldL = l;
% oldR = r;
i=i+1;
j=j+1;
l(i) = obj.encoders.LatestMessage.Left;
r(i) = obj.encoders.LatestMessage.Right;
t(i) = obj.LatestMessage.Header.Stamp.Sec + (obj.LatestMessage.Header.Stamp.Nsec / 1000000000.0);
dl(j)=l(i)-l(i-1);
dr(j)=r(i)-r(i-1);
dm(j)=(dl(j)+dr(j))/2;
theta(j)=(dr(j)-dl(j))/w;
x(j)=dm(j)*cos(pi/2-theta(j)/2-sum)+x(j);
y(j)=dm(j)*sin(pi/2-sum)+y(j);
if j==1
  sumx(j)=x;
  sumy(j)=y;
  sum=theta(j)+sum;
else
  sumx(j)=sumx(j-1)+x;
  sumy(j)=sumy(j-1)+y;
  sum=theta(j)+sum;
end

% l = robot.encoders.LatestMessage.Left;
% r = robot.encoders.LatestMessage.Right;

% if oldL ~= l || oldR~= r
%     oldTime = time;
%     time = toc - time0;


    %time1 =  robot.encoders.LatestMessage.Header.Stamp.Sec
    %time2 =  robot.encoders.LatestMessage.Header.Stamp.Nsec
   % reading = 1;
end
