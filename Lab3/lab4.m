global l r;
global t;
global i;
global j;
global dr;
global dl;
global w;
global sum;
global theta;
global dm;
global x;
global y;
global sumx;
global sumy;
aa=zeros(1,1);
sumx=zeros(1,1);
sumy=zeros(1,1);
hh=zeros(1,1);
sum=0;
w=235;
l = zeros(1,1);
r = zeros(1,1);
dr=zeros(1,1);
dl=zeros(1,1);
t=zeros(1,1);
theta=zeros(1,1);
dm=zeros(1,1);
%neatoEncoderFrameee = 0;
%enc=rossubscriber('/enc',@newEncoderDataCallback);
%robot.encoders.NewMessageFcn=@neatoEncoderEventListener;
%Encoders=receive(enc);
kv =.4;
ks = .5;
i=1;
j=0;
t(i) = 0;
l(i)=robot.encoders.LatestMessage.Left;
r(i)=robot.encoders.LatestMessage.Right;
tic;
while t(i) < 12.565*ks/kv

    vr = 0.3*kv + 0.14125*kv/ks*sin(t(i)*kv/2/ks);
    vl = 0.3*kv - 0.14125*kv/ks*sin(t(i)*kv/2/ks);
    robot.sendVelocity(vl,vr);
    pause(.01);
%     le;
%     re;clear 
    i=i+1;
j=j+1;
l(i) = robot.encoders.LatestMessage.Left;
r(i) = robot.encoders.LatestMessage.Right;
t(i) = toc;
dl(j)=l(i)-l(i-1);
dr(j)=r(i)-r(i-1);
dm(j)=(dl(j)+dr(j))/2;
theta(j)=(-dl(j)+dr(j))/w;
x=dm(j)*cos(pi/2-theta(j)/2-sum);
y=dm(j)*sin(pi/2-theta(j)/2-sum);
% x=dm(j)*sin(theta(j)/2+sum);
% y=dm(j)*cos(theta(j)/2+sum);
if j==1
  sumx(j)=x;
  sumy(j)=y;
  sum=theta(j)+sum;
else
  sumx(j)=sumx(j-1)+x;
  sumy(j)=sumy(j-1)+y;
  sum=theta(j)+sum;
end
hh(j)=sum;
aa(j)=j;
end
figure(1);
plot(aa,hh);
figure(2);
plot(sumx,sumy);

robot.shutdown();
