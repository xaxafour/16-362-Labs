%robot.startLaser();
%pause(5);
lines_p1=[0,1.22; 0,0];
lines_p2=[0,0; 1.22,0];
robotEstPose = pose(0.5,0.5,pi/2.0);
% control the scale of the diff between S(fused) and S(map)
% change according to the movement of robot
k1 = 0.1;
k2 = 0.5;
k3 = 0.2;

delay = 0.3;
Vmax = 0.2;

% get refined initial guess
localizer = lineMapLocalizer(robot, lines_p1,lines_p2,0.1,0.001,0.0005);
[success,RobotStPose] = localizer.initpos(robotEstPose);

system = mrplSystem(robot, RobotStPose, localizer);
p1Controller = controller(0.1, 1);
p2Controller = controller(1, 1);
p3Controller = controller(0.5, 1);

pose1 = pose(0.25, 0.75, pi/2.0);
pose2 = pose(0.75, 0.25, 0.0);
pose3 = pose(0.5, 0.5, pi/2.0);

system.executeTrajectory(pose1, p1Controller, delay, 0.15, 1, k1);
pause(2);
system.executeTrajectory(pose2, p2Controller, delay, Vmax, 1, k2);
pause(2);
system.executeTrajectory(pose3, p3Controller, delay, Vmax, 1, k3);