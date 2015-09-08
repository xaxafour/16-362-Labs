%% Connect to the Robot
robot = neato('yotta');

signedDistance = 0;
timeArray=zeros(1,1);
distArray=zeros(1,1);
i = 1;
robot.sendVelocity(0, 0)
tic
leftStart = robot.encoders.LatestMessage.Left / 10;
figure(1)
h = animatedline;
title('Lab 1')
xlabel('Time (sec)')
ylabel('Position (cm)')
%% Move the Neato forward and backward

while signedDistance-leftStart < 20
    % Look at the encoder data of the left wheel (in mm)
    leftEncoder = robot.encoders.LatestMessage.Left;
    signedDistance = leftEncoder / 10;
    if signedDistance-leftStart < 19
        robot.sendVelocity(.05, .05)
    else
        robot.sendVelocity(.02, .02)
    end
    timeArray(i) = toc;
    distArray(i) = signedDistance-leftStart;
    addpoints(h,timeArray(i),distArray(i))
    drawnow
    i = i+1;
    pause(0.005)
end

robot.sendVelocity(0, 0)
pause(2)

while signedDistance-leftStart > 0
    % Look at the encoder data of the left wheel (in mm)
    leftEncoder = robot.encoders.LatestMessage.Left;
    signedDistance = leftEncoder / 10;
    if signedDistance-leftStart > 1
        robot.sendVelocity(-.05, -.05)
    else
        robot.sendVelocity(-.02, -.02)
    end
    timeArray(i) = toc;
    distArray(i) = signedDistance-leftStart;
    addpoints(h,timeArray(i),distArray(i))
    drawnow
    i = i+1;
    pause(0.005)
end

robot.sendVelocity(0, 0)
pause(1)
robot.close()