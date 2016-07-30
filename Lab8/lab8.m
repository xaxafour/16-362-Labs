
startPose = pose(0, 0, 0);
system = mrplSystem(robot, startPose);
pController = controller(0.5, 2);
delay = 0.3;
Vmax = 0.2;

%robot.startLaser();
%pause(2);
%calculate sensor to object
ranges = robot.laser.LatestMessage.Ranges;
image = rangeImage(ranges, 1, 1);
mx = image.middlex;
my = image.middley;
mth = image.theta;
mthDeg = mth * 180 / pi;

%this is magic
sensorToObjectTh = atan2(my, mx);
sensorToObjectDist = sqrt((mx^2) + (my^2));
yoffset = (sensorToObjectTh/pi) * 0.08;
xoffset = sensorToObjectDist * 0.08;

%homogeneous transforms to get from robot to object
Trs = pose(robotModel.laser_l, 0, 0);
Tso = pose(mx, my, mth);
Tog = pose(-robotModel.objOffset-robotModel.frontOffset + xoffset, 0 + yoffset, 0);
Trg = Trs.bToA * Tso.bToA * Tog.bToA;

pose1 = pose(pose.matToPoseVec(Trg));
targetx = pose1.x - robotModel.laser_l;
targety = pose1.y;
targetth = pose1.th * 180 / pi;

%system.executeTrajectory(pose1, pController, delay, Vmax, 1);