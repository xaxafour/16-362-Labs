
system = mrplSystem(robot);
pController = controller(2, 12);
delay = 0.3;
Vmax = 0.2;

curve1 = cubicSpiral.planTrajectory(0.25, 0.25, 0, 1);
system.executeTrajectory(curve1, pController, delay, Vmax, 1);
pause(2);

curve2 = cubicSpiral.planTrajectory(-0.5, -0.5, -pi/2, 1);
system.executeTrajectory(curve2, pController, delay, Vmax, 2);
pause(2);

curve3 = cubicSpiral.planTrajectory(-0.25, 0.25, pi/2, 1);
system.executeTrajectory(curve3, pController, delay, Vmax, 3);

