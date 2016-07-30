robot = 0;
%robot = neato('nano');

startPose = pose(0, 0, 0);
system = mrplSystem(robot, startPose);
pController = controller(0.5, 0.1);
delay = 0.3;
Vmax = 0.2;

pose1 = pose(0.25, 0.25, 0);
pose2 = pose(-0.5, -0.5, -pi/2);
pose3 = pose(-0.25, 0.25, pi/2);


system.executeTrajectory(pose1, pController, delay, Vmax, 1);
pause(2);
system.executeTrajectory(pose2, pController, delay, Vmax, 1);
pause(2);
system.executeTrajectory(pose3, pController, delay, Vmax, 1);



