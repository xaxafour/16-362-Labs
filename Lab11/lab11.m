% robot.startLaser();
% pause(5);

lines_p1 = [0,1.22; 0,0];
lines_p2 = [0,0; 1.22,0];
robotEstPose = pose(0.5, 0.5, pi/2);
% control the scale of the diff between S(fused) and S(map)
% change according to the movement of robot
k = 0.3;

delay = 0.3;
Vmax = 0.2;
useLaser = 1;

% get refined initial guess
localizer = lineMapLocalizer(robot, lines_p1,lines_p2,0.1,0.001,0.0005);
if useLaser
    [success, robotStPose] = localizer.initpos(robotEstPose);
else
    success = 1;
    robotStPose = robotEstPose;
end

% display(robotStPose.x);
% display(robotStPose.y);
% display(robotStPose.th);

system = mrplSystem(robot, robotStPose, localizer);
% p1Controller = controller(0.5, 1);
% p2Controller = controller(0.5, -1);

 p1Controller = controller(1.2, 2.4,0.4);
 p2Controller = controller(0.5, -1,0.3);

for i = 1:3
    %object position relative to sensor
    ranges = robot.laser.LatestMessage.Ranges;
    %ranges(60:300) = 0;
    image = rangeImage(ranges, 1, 1, robotStPose);
    Tso = pose(image.middlex, image.middley, image.theta);
%     display(Tso.x);
%     display(Tso.y);
%     display(Tso.th);

    %homogeneous transforms from world to goal frame
    Twr = system.encCurrentPose;
    Trs = pose(robotModel.laser_l, 0, 0);
    Tog = pose(-robotModel.objOffset-robotModel.frontOffset, 0, 0);
    if i==3
       Tso = pose(image.middlex+0.01, image.middley-0.01, image.theta);
    end
    if i==2
       Tso = pose(image.middlex+0.03, image.middley, image.theta);
    end
    pose1 = pose(pose.matToPoseVec(Twr.bToA * Trs.bToA * Tso.bToA * Tog.bToA));

    system.loadTrajectory('cubic', pose1);
    system.executeTrajectory(pose1, p1Controller, delay, Vmax, 1, k, useLaser);
% %     pause(1);
    
    system.resetReference();
    pose2 = system.loadTrajectory('straight', [0.15, Vmax, -1]);
    system.executeTrajectory(pose2, p2Controller, delay, Vmax, 1, k, useLaser);
%     pause(1);
    
    system.resetReference();
    pose3 = system.loadTrajectory('rotate', [pi, Vmax*4, 1]);
    system.executeTrajectory(pose3, p1Controller, delay, Vmax, 1, k, useLaser);
    
    system.resetReference();
    pause(30);
end
% robot.stopLaser();








