global firstEncoder encoderL0 encoderR0 encoderT0 encoderL encoderR encoderT i 
encoderL0 = 0;
encoderR0 = 0;
encoderT0 = 0;
encoderL = 0;
encoderR = 0;
encoderT = 0;
firstEncoder = true;

usePID = true;

if usePID
    controller = figure8ReferenceControl(0.4, 0.4, 2);
else
    controller = figure8ReferenceControl(0.4, 0.4, 1);
end
trajectory = robotTrajectory(controller, 1000);

el=zeros(1,1);  %left encoder 
er = zeros(1,1);
del= zeros(1,1);
der= zeros(1,1);
drel = zeros(1,1);
drer = zeros(1,1);
rel = zeros(1,1);
rer = zeros(1,1);
t = zeros(1,1);
et = zeros(1,1);
dt = zeros(1,1);
edt = zeros(1,1);
uv = zeros(1,1);
uw = zeros(1,1);
thetarobot=zeros(1,1);
x = zeros(1,1);
y = zeros(1,1);
theta = zeros(1,1);
referx = zeros(1,1);
refery = zeros(1,1);
rtheta = zeros(1,1);
robt= controller.getTrajectoryDuration();%get the end time
i = 1;
t(1)=0;
x(1)=0;   % world frame position y
y(1)=0;%world frame position x 
loops = 0;
j = 1;
theta(1) = 0;
rtheta(1) = 0;

%error declare
error = zeros(2, 1);
derror=[0;0];
ierror(1,1)=0;
ierror(2,1)=0;
kpx=0.01;
kpy=0.06;
kdx=0;
kdy=0;
kix=1;
kiy=0.1;
delay = 0.3;

dvl = 0;
dvr = 0;

while et(j) < robt 
    if loops < 2
        robot.sendVelocity(0, 0);
    elseif loops == 2
        robot.encoders.NewMessageFcn = @newEncoderDataCallback;
        pause(1);
        %c = clock;
        %startTime = c(5)*60 + c(6);
    else
        et(i) = encoderT; %real and refer
        %c = clock;
        %t(i) = c(5)*60 + c(6) - startTime;
        el(i) = encoderL; %real
        er(i) = encoderR; %real

        if i > 1
            edt(j)=et(i)-et(i-1);%%real and refer use together
            %dt(j)=t(i)-t(i-1);
            del(j) = el(i)-el(i-1);%%real
            der(j) =er(i)-er(i-1);%%real
            if edt(j) ~= 0
                dvl = del(j)/edt(j);
                dvr = der(j)/edt(j);
            end
            [dv, dw] = robotModel.vlvrToVw(dvl, dvr);
            dth = dw*edt(j);
            ds = dv*edt(j);

            theta(i) = theta(i-1)+dth; %%theta is the sum of dth between every 2points
            dx = cos(theta(i))*ds;
            dy = sin(theta(i))*ds;
            x(i) = x(i-1)+dx; 
            y(i) = y(i-1)+dy;

            rtheta(i) = trajectory.getPoseAtTime(et(i)-delay).th;
            referx(i) = trajectory.getPoseAtTime(et(i)-delay).x;
            refery(i) = trajectory.getPoseAtTime(et(i)-delay).y;
            % here we can get the x,y of both reference and real
            j=j+1;
        end
        %now deal with the error
        error(:,i) = ([cos(theta(i)),-sin(theta(i));sin(theta(i)),cos(theta(i))])\[referx(i)-x(i); refery(i)-y(i)];
        %world frame error  the theta here is the error at this point
        %thetarobot(i) = atan(error(1,i)/error(2,i));
        %if i > 1  
        %  derror=( error(:,i)-error(:,i-1))/(t(i)-t(i-1)); %%is it the / or ./
        %  ierror(:,i)= ierror(:,i-1) + error(:,i) * (t(i)-t(i-1));
        %end
        v = trajectory.getLinVelAtTime(et(i));
        w = trajectory.getAngVelAtTime(et(i));
        
        if usePID
            uv(i) = v+ kpx * error(1,i); %+ kix * ierror(1,i) + kdx* derror(1);
            uw(i) = w+ kpy * error(2,i); %+ kiy * ierror(2,i) + kdy* derror(2);
        else
            uv(i) = v;
            uw(i) = w;
        end
        
        %up=[kx,0;0,ky]*error;
        %uv=v+ up(1,1);  %%v comes from figure8ReferenceControl
        %uw = w+ up(2,1); %%w comes from figure8ReferenceControl
        
        
        %%use the VwTovlvr in class robotModel to tanslate uv uw into vl
        %%vr(may be another name)
        [vl, vr] = robotModel.VwTovlvr(uv(i), uw(i));
        [vl, vr] = robotModel.limitWheelVelocities([vl, vr]);
        
        robot.sendVelocity(vl, vr);
        i=i+1;
    end
    loops = loops+1;

    pause(0.1);
end

figure(4);
plot(et, x, et, y, et, theta, et, referx, et, refery, et, rtheta);
figure(3);
plot(x, y, referx, refery);
%figure(3);
%plot(t, x-referx, t, y-refery, t, theta-rtheta);
figure(1);
plot(et,error(1,:));
figure(2);
plot(et,error(2,:));
finalerror=(error(1,i-1)^2+error(2,i-1)^2)^0.5;%%final error is the abs one
display(finalerror);
