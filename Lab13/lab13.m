% robot.startLaser();
% pause(5);


robot.encoders.NewMessageFcn = @newEncoderDataCallback;
robot.laser.NewMessageFcn = @neatoLaserEventListener;
global neatoLaserRanges;

lines_p1 = [0,1.8288; 0,0];
lines_p2 = [0,0; 1.8288,0];
%robotEstPose = pose(0.2286, 0.2286, -pi/2);
% control the scale of the diff between S(fused) and S(map)
k = 0.35;
k1 = 0.35;
k2 = 0.35;
k3 = 0.35;

delay = 0.3;
Vmax = 0.25;
useLaser = 1;
notuseLaser = 0;

% get refined initial guess
localizer = lineMapLocalizer(robot, lines_p1,lines_p2,0.1,0.001,0.0005);
% if useLaser
%     [success, robotStPose] = localizer.initpos(robotEstPose);
% else
%     success = 1;
%     robotStPose = robotEstPose;
% end

% display(robotStPose.x);
% display(robotStPose.y);
% display(robotStPose.th);


p1Controller = controller(1, 1,0.5);
p2Controller = controller(0.5, -1,0.3);
p3Controller = controller(1.2, 1.2,0.5);


t1=0.7;
t2=0.8;
t5=0.8;
t4=0.6;
t3=0.7;
t6=0.6;
t7=1.05;
%first rotate
% robot.sendVelocity(-0.25,0.25);
% pause(t1);
% robot.sendVelocity(-0.25,0.25);
% pause(t2);
% robot.sendVelocity(0,0);

robotEstPose = pose(0.25,0.25,-pi/2);
if useLaser
    [success, robotStPose] = localizer.initpos(robotEstPose);
else
    success = 1;
    robotStPose = robotEstPose;
end

system = mrplSystem(robot, robotStPose, localizer);

pose1ro=system.loadTrajectory('fl_rotate', [pi-0.1, Vmax*6, 1]);
%pose1ro=system.loadTrajectory('fl_rotate', [-pi/2, Vmax*8, 1]);
%pose2ro=pose(robotStPose.x,robotStPose.y,0.4767);
system.executeTrajectory(pose1ro, p3Controller, delay, Vmax, 1, k2, useLaser);
%
robotEstPose = system.encCurrentPose;
if useLaser
    [success, robotStPose] = localizer.initpos(robotEstPose);
else
    success = 1;
    robotStPose = robotEstPose;
end
%system = mrplSystem(robot, robotStPose, localizer);
%go to first ready place
pose1r=pose(0.3048,1.1,pi/2);
system.loadTrajectory('cubic', pose1r);
system.executeTrajectory(pose1r, p3Controller, delay, Vmax, 1, k2, useLaser);
%go to pick first pallet
ranges = neatoLaserRanges;
image = rangeImage(ranges, 1, 1, system.encCurrentPose);

if image.objsucflag==0
    robot.sendVelocity(0.1,0.1);
    pause(0.2);
    robot.sendVelocity(0,0);
    ranges = neatoLaserRanges;
    image = rangeImage(ranges, 1, 1, system.encCurrentPose);
end

ft=1;
while image.objsucflag==0
    robot.sendVelocity(0.1,-0.1);
    pause(0.2);
    robot.sendVelocity(0,0);
    ranges = neatoLaserRanges;
    image = rangeImage(ranges, 1, 1, system.encCurrentPose);
    ft=ft+1;
    if(ft==5)
        break;
    end
end

if ft<5
    Tso = pose(image.middlex+0.03, image.middley, image.theta);
    % display(Tso.x);
    % display(Tso.y);
    % display(Tso.th);
    %homogeneous transforms from world to goal frame
    Twr = system.encCurrentPose;
    Trs = pose(robotModel.laser_l, 0, 0);
    Tog = pose(-robotModel.objOffset-robotModel.frontOffset, 0, 0);
    pose1p = pose(pose.matToPoseVec(Twr.bToA * Trs.bToA * Tso.bToA * Tog.bToA));
    display(pose1p);
    system.loadTrajectory('cubic', pose1p);
    system.executeTrajectory(pose1p, p1Controller, delay, Vmax, 1, k1, useLaser);
    robot.forksUp();
    pause(0.3);
    
    %  pose1b = system.loadTrajectory('straight', [0.05, 0.2, -1]);
    %  system.executeTrajectory(pose1b, p2Controller, delay, Vmax, 1, k, useLaser,1);
    
    pose1b = system.loadTrajectory('straight', [0.05, 0.25, -1]);
    system.executeTrajectory(pose1b, p2Controller, delay, Vmax, 1, k, useLaser);
    %display(RobPose);    %take a look at this one!! is it right ?!
    %pause(0.5);
    pose1ro=system.loadTrajectory('fl_rotate', [pi, Vmax*6, -1]);
    %pose1ro=system.loadTrajectory('fl_rotate', [-pi/2, Vmax*8, 1]);
    %pose2ro=pose(robotStPose.x,robotStPose.y,0.4767);
    system.executeTrajectory(pose1ro, p3Controller, delay, Vmax, 1, k2, useLaser);
    
    
    pose1d=pose(0.3048-0.05,0.5048,-pi/2);
    system.loadTrajectory('cubic', pose1d);
    system.executeTrajectory(pose1d, p1Controller, delay, Vmax, 1, k1, useLaser);
    robot.forksDown();
    pause(0.3);
    
    pose1bb = system.loadTrajectory('straight', [0.05, 0.25, -1]);
    system.executeTrajectory(pose1bb, p2Controller, delay, Vmax, 1, k, useLaser);
else
    pose1bb = system.loadTrajectory('straight', [0.4, 0.25, -1]);
    system.executeTrajectory(pose1bb, p2Controller, delay, Vmax, 1, k, useLaser);
end
% robot.sendVelocity(-0.25,-0.25);
% pause(t6);   % make robot go back about 0.3m
% robot.sendVelocity(0,0);

% display(system.encCurrentPose.x);
% display(system.encCurrentPose.y);
% display(system.encCurrentPose.th);


for i=1:2
    %rotate after first drop
    % robotEstPose=pose((0.3048)*i,0.8048,-pi/2);
    % if useLaser
    %     [success, robotStPose] = localizer.initpos(robotEstPose);
    % else
    %     success = 1;
    %     robotStPose = robotEstPose;
    % end
    % %system = mrplSystem(robot, robotStPose, localizer);
    % system.encCurrentPose=robotStPose;
    % system.refCurrentPose=robotStPose;
    %ranges = neatoLaserRanges;
    %if isempty(ranges)
    %    display('laser goes wrong!!');
    %end
    %pose2ro=system.loadTrajectory('fl_rotate', [0.4767, Vmax*8, 1]);
    pose2ro=system.loadTrajectory('fl_rotate', [0.4767+pi/2, Vmax*6, 1]);
    %pose2ro=pose(robotStPose.x,robotStPose.y,0.4767);
    system.executeTrajectory(pose2ro, p3Controller, delay, Vmax, 1, k2, useLaser);
    %go to second pick up place
    pose2r=pose((0.3048)*(i+1),1.1,pi/2);
    system.loadTrajectory('cubic', pose2r);
    system.executeTrajectory(pose2r, p3Controller, delay, Vmax, 1, k2, useLaser);
    
    %display(system.encCurrentPose);
    
    %go to pick up second pallet
    ranges = neatoLaserRanges;
    image = rangeImage(ranges, 1, 1, system.encCurrentPose);
    
    if image.objsucflag==0
        robot.sendVelocity(0.1,0.1);
        pause(0.2);
        robot.sendVelocity(0,0);
        ranges = neatoLaserRanges;
        image = rangeImage(ranges, 1, 1, system.encCurrentPose);
    end
    
    ft=1;
    while image.objsucflag==0
        robot.sendVelocity(0.1,-0.1);
        pause(0.2);
        robot.sendVelocity(0,0);
        ranges = neatoLaserRanges;
        image = rangeImage(ranges, 1, 1, system.encCurrentPose);
        ft=ft+1;
        if(ft==5)
            break;
        end
    end
    
    if ft<5
        Tso = pose(image.middlex+0.03, image.middley, image.theta);
        % display(Tso.x);
        % display(Tso.y);
        % display(Tso.th);
        %homogeneous transforms from world to goal frame
        Twr = system.encCurrentPose;
        Trs = pose(robotModel.laser_l, 0, 0);
        Tog = pose(-robotModel.objOffset-robotModel.frontOffset, 0, 0);
        pose2p = pose(pose.matToPoseVec(Twr.bToA * Trs.bToA * Tso.bToA * Tog.bToA));
        system.loadTrajectory('cubic', pose2p);
        system.executeTrajectory(pose2p, p1Controller, delay, Vmax, 1, k1, useLaser);
        robot.forksUp();
        pause(0.3);
        % RobPose=system.s_str_rot(t3,t4,t5,i+1);
        % display(RobPose);    %take a look at this one!! is it right ?!
        
        pose2b = system.loadTrajectory('straight', [0.05, 0.2, -1]);
        system.executeTrajectory(pose2b, p2Controller, delay, Vmax, 1, k, useLaser);
        
        pose1ro=system.loadTrajectory('fl_rotate', [pi, Vmax*6, -1]);
        %pose1ro=system.loadTrajectory('fl_rotate', [-pi/2, Vmax*8, 1]);
        %pose2ro=pose(robotStPose.x,robotStPose.y,0.4767);
        system.executeTrajectory(pose1ro, p3Controller, delay, Vmax, 1, k2, useLaser);
        
        pose2d=pose((0.3048)*(i+1),0.5048,-pi/2);
%         if i==1
%            pose2d=pose((0.3048)*(i+1),0.5048-0.1,-pi/2);
%         end
        system.loadTrajectory('cubic', pose2d);
        system.executeTrajectory(pose2d, p1Controller, delay, Vmax, 1, k1, useLaser);
        robot.forksDown();
        pause(0.3);
        
        pose22b = system.loadTrajectory('straight', [0.05, 0.2, -1]);
        system.executeTrajectory(pose22b, p2Controller, delay, Vmax, 1, k, useLaser);
        
    else
        pose22b = system.loadTrajectory('straight', [0.4, 0.2, -1]);
        system.executeTrajectory(pose22b, p2Controller, delay, Vmax, 1, k, useLaser);
    end
    % robot.sendVelocity(-0.25,-0.25);
    % pause(t6);   % make robot go back about 0.3m
    % robot.sendVelocity(0,0);
end
%
%rotate after third drop
% robotEstPose=pose(0.3048*3,0.8048,-pi/2);
% if useLaser
%     [success, robotStPose] = localizer.initpos(robotEstPose);
% else
%     success = 1;
%     robotStPose = robotEstPose;
% end
% system.encCurrentPose=robotStPose;
% system.refCurrentPose=robotStPose;
pose4ro=system.loadTrajectory('fl_rotate', [0.4767+pi/2, Vmax*6, 1]);
system.executeTrajectory(pose4ro, p3Controller, delay, Vmax, 1, k2, useLaser);

%check if the pallet has been taken by some one

%go to fourth pick up place
pose4r=pose(0.3048*4,1.1,pi/2);
system.loadTrajectory('cubic', pose4r);
system.executeTrajectory(pose4r, p3Controller, delay, Vmax, 1, k2, useLaser);

%go to pick up fourth pallet
ranges = neatoLaserRanges;
image = rangeImage(ranges, 1, 1, system.encCurrentPose);

if image.objsucflag==0
    robot.sendVelocity(0.1,0.1);
    pause(0.2);
    robot.sendVelocity(0,0);
    ranges = neatoLaserRanges;
    image = rangeImage(ranges, 1, 1, system.encCurrentPose);
end

ft=1;
while image.objsucflag==0
    robot.sendVelocity(0.1,-0.1);
    pause(0.2);
    robot.sendVelocity(0,0);
    ranges = neatoLaserRanges;
    image = rangeImage(ranges, 1, 1, system.encCurrentPose);
    ft=ft+1;
    if(ft==5)
        break;
    end
end

if ft<5
    
    Tso = pose(image.middlex+0.03, image.middley-0.01, image.theta);
    % display(Tso.x);
    % display(Tso.y);
    % display(Tso.th);
    %homogeneous transforms from world to goal frame
    Twr = system.encCurrentPose;
    Trs = pose(robotModel.laser_l, 0, 0);
    Tog = pose(-robotModel.objOffset-robotModel.frontOffset, 0, 0);
    pose4p = pose(pose.matToPoseVec(Twr.bToA * Trs.bToA * Tso.bToA * Tog.bToA));
    system.loadTrajectory('cubic', pose4p);
    system.executeTrajectory(pose4p, p1Controller, delay, Vmax, 1, k1, useLaser);
    robot.forksUp();
    pause(0.3);
    pose4b = system.loadTrajectory('straight', [0.05, 0.2, -1]);
    system.executeTrajectory(pose4b, p2Controller, delay, Vmax, 1, k, useLaser);
    %display(RobPose);    %take a look at this one!! is it right ?!
    
    pose4ro=system.loadTrajectory('fl_rotate', [pi, Vmax*6, -1]);
    %pose1ro=system.loadTrajectory('fl_rotate', [-pi/2, Vmax*8, 1]);
    %pose2ro=pose(robotStPose.x,robotStPose.y,0.4767);
    system.executeTrajectory(pose4ro, p3Controller, delay, Vmax, 1, k2, useLaser);
    
    
    pose4d=pose(1.524,0.5048,-pi/2);
    system.loadTrajectory('cubic', pose4d);
    system.executeTrajectory(pose4d, p1Controller, delay, Vmax, 1, k1, useLaser);
    robot.forksDown();
    pause(0.3);
    % after pick up four forward pallet, begin to pick up the three on on side
    %
    %
    %
    %
    % %rotate to face the first side pallet
    
    % robotEstPose=pose(0.4244,1.5795,-pi/2);
    %
    % if useLaser
    %     [success, robotStPose] = localizer.initpos(robotEstPose);
    % else
    %     success = 1;
    %     robotStPose = robotEstPose;
    % end
    %  system = mrplSystem(robot, robotStPose, localizer);
    
    
    % pose5b = system.loadTrajectory('straight', [0.08, 0.2, -1]);
    % system.executeTrajectory(pose5b, p2Controller, delay, Vmax, 1, k, useLaser,1);
    
    %
    pose5ro=system.loadTrajectory('fl_rotate', [pi/2-0.3, Vmax*6, 1]);
    system.executeTrajectory(pose5ro, p3Controller, delay, Vmax, 1, k2, useLaser);
    
    
    
    pose14 = system.loadTrajectory('straight', [0.3, 0.25, -1]);
    system.executeTrajectory(pose14, p2Controller, delay, Vmax, 1, k, useLaser);
    
else
    pose14 = system.loadTrajectory('straight', [0.4, 0.25, -1]);
    system.executeTrajectory(pose14, p2Controller, delay, Vmax, 1, k, useLaser);
end
pose5r=pose(1.6,0.55,0);
system.loadTrajectory('cubic', pose5r);
system.executeTrajectory(pose5r, p1Controller, delay, Vmax, 1, k1, useLaser);

%go to pick up the first side pallet
ranges = neatoLaserRanges;
image = rangeImage(ranges, 1, 1, system.encCurrentPose);

if image.objsucflag==0
    robot.sendVelocity(0.1,0.1);
    pause(0.2);
    robot.sendVelocity(0,0);
    ranges = neatoLaserRanges;
    image = rangeImage(ranges, 1, 1, system.encCurrentPose);
end

ft=1;
while image.objsucflag==0
    robot.sendVelocity(0.1,-0.1);
    pause(0.2);
    robot.sendVelocity(0,0);
    ranges = neatoLaserRanges;
    image = rangeImage(ranges, 1, 1, system.encCurrentPose);
    ft=ft+1;
    if(ft==5)
        break;
    end
end

if ft<5
    
    Tso = pose(image.middlex+0.03, image.middley-0.01, image.theta);  % over a little to make sure the robot can rotate to drop
    % display(Tso.x);
    % display(Tso.y);
    % display(Tso.th);
    %homogeneous transforms from world to goal frame
    Twr = system.encCurrentPose;
    Trs = pose(robotModel.laser_l, 0, 0);
    Tog = pose(-robotModel.objOffset-robotModel.frontOffset, 0, 0);
    pose5p = pose(pose.matToPoseVec(Twr.bToA * Trs.bToA * Tso.bToA * Tog.bToA));
    system.loadTrajectory('cubic', pose5p);
    system.executeTrajectory(pose5p, p1Controller, delay, Vmax, 1, k1, useLaser);
    robot.forksUp();
    pause(0.3);
    
    %rotate to drop
    pose5dro=system.loadTrajectory('fl_rotate', [pi/2-1.2, Vmax*6, -1]);
    system.executeTrajectory(pose5dro, p3Controller, delay, Vmax, 1, k2, useLaser);  % if don't need to plan another cubic set last parameter to 0
    
    pose5d=pose(2.1336,0.5048-0.1,-pi/2);
    system.loadTrajectory('cubic', pose5d);
    system.executeTrajectory(pose5d, p3Controller, delay, Vmax, 1, k1, useLaser);
    robot.forksDown();
    pause(0.3);
    pose14 = system.loadTrajectory('straight', [0.05, 0.25, -1]);
    system.executeTrajectory(pose14, p2Controller, delay, Vmax, 1, k, useLaser);
    
    
    pose6ro=system.loadTrajectory('fl_rotate', [pi/2-0.5, Vmax*6, 1]);
    system.executeTrajectory(pose6ro, p3Controller, delay, Vmax, 1, k2, useLaser);
    
    % pose6back = system.loadTrajectory('straight', [1.2192, 0.28, -1]);
    % system.executeTrajectory(pose6back, p2Controller, delay, Vmax, 1, k, useLaser,1);
    %
    %go to the sixth pick up place
else
    pose15 = system.loadTrajectory('straight', [0.4, 0.25, -1]);
    system.executeTrajectory(pose15, p2Controller, delay, Vmax, 1, k, useLaser);
end

pose15 = system.loadTrajectory('straight', [0.8, 0.25, -1]);
system.executeTrajectory(pose15, p2Controller, delay, Vmax, 1, k, useLaser);

pose6r=pose(1.60,1.18,0);
system.loadTrajectory('cubic', pose6r);
system.executeTrajectory(pose6r, p3Controller, delay, Vmax, 1, k2, useLaser);

%go to pick up 6th pallet
ranges = neatoLaserRanges;
image = rangeImage(ranges, 1, 1, system.encCurrentPose);

if image.objsucflag==0
    robot.sendVelocity(0.1,0.1);
    pause(0.2);
    robot.sendVelocity(0,0);
    ranges = neatoLaserRanges;
    image = rangeImage(ranges, 1, 1, system.encCurrentPose);
end

ft=1;
while image.objsucflag==0
    robot.sendVelocity(0.1,-0.1);
    pause(0.2);
    robot.sendVelocity(0,0);
    ranges = neatoLaserRanges;
    image = rangeImage(ranges, 1, 1, system.encCurrentPose);
    ft=ft+1;
    if(ft==5)
        break;
    end
end

if ft<5
    Tso = pose(image.middlex+0.03, image.middley, image.theta);
    display(Tso.x);
    display(Tso.y);
    display(Tso.th);
    %homogeneous transforms from world to goal frame
    Twr = system.encCurrentPose;
    Trs = pose(robotModel.laser_l, 0, 0);
    Tog = pose(-robotModel.objOffset-robotModel.frontOffset, 0, 0);
    pose6p = pose(pose.matToPoseVec(Twr.bToA * Trs.bToA * Tso.bToA * Tog.bToA));
    system.loadTrajectory('cubic', pose6p);
    system.executeTrajectory(pose6p, p1Controller, delay, Vmax, 1, k1, useLaser);
    robot.forksUp();
    pause(0.3);
    
    pose6b = system.loadTrajectory('straight', [0.05, 0.25, -1]);
    system.executeTrajectory(pose6b, p2Controller, delay, Vmax, 1, k, useLaser);
    
    
    pose6ro=system.loadTrajectory('fl_rotate', [pi/2, Vmax*6, -1]);
    system.executeTrajectory(pose6ro, p3Controller, delay, Vmax, 1, k2, useLaser);
    
    pose6d=pose(1.25,0.5048-0.1,-pi/2);
    system.loadTrajectory('cubic', pose6d);
    system.executeTrajectory(pose6d, p1Controller, delay, Vmax, 1, k1, useLaser);
    robot.forksDown();
    pause(0.3);
    
    pose6bb = system.loadTrajectory('straight', [0.05, 0.25, -1]);
    system.executeTrajectory(pose6bb, p2Controller, delay, Vmax, 1, k, useLaser);
    
    
    pose7ro=system.loadTrajectory('fl_rotate', [pi, Vmax*6, 1]);
    system.executeTrajectory(pose7ro, p3Controller, delay, Vmax, 1, k2, useLaser);
else
    pose6bb = system.loadTrajectory('straight', [0.6, 0.25, -1]);
    system.executeTrajectory(pose6bb, p2Controller, delay, Vmax, 1, k, useLaser);
end
%7th ready place
pose7r=pose(1.524,0.8,0);
system.loadTrajectory('cubic', pose7r);
system.executeTrajectory(pose7r, p3Controller, delay, Vmax, 1, k2, useLaser);

%7th pick-up place

ranges = neatoLaserRanges;
image = rangeImage(ranges, 1, 1, system.encCurrentPose);

if image.objsucflag==0
    robot.sendVelocity(0.1,0.1);
    pause(0.2);
    robot.sendVelocity(0,0);
    ranges = neatoLaserRanges;
    image = rangeImage(ranges, 1, 1, system.encCurrentPose);
end


ft=1;
while image.objsucflag==0
    robot.sendVelocity(0.1,-0.1);
    pause(0.2);
    robot.sendVelocity(0,0);
    ranges = neatoLaserRanges;
    image = rangeImage(ranges, 1, 1, system.encCurrentPose);
    ft=ft+1;
    if(ft==5)
        break;
    end
end

Tso = pose(image.middlex+0.03, image.middley, image.theta);
% display(Tso.x);
% display(Tso.y);
% display(Tso.th);
%homogeneous transforms from world to goal frame
Twr = system.encCurrentPose;
Trs = pose(robotModel.laser_l, 0, 0);
Tog = pose(-robotModel.objOffset-robotModel.frontOffset, 0, 0);
pose7p = pose(pose.matToPoseVec(Twr.bToA * Trs.bToA * Tso.bToA * Tog.bToA));
system.loadTrajectory('cubic', pose7p);
system.executeTrajectory(pose7p, p1Controller, delay, Vmax, 1, k1, useLaser);
robot.forksUp();
pause(0.3);

pose7b = system.loadTrajectory('straight', [0.05, 0.25, -1]);
system.executeTrajectory(pose7b, p2Controller, delay, Vmax, 1, k, useLaser);


pose7ro=system.loadTrajectory('fl_rotate', [pi/2, Vmax*6, -1]);
system.executeTrajectory(pose7ro, p3Controller, delay, Vmax, 1, k2, useLaser);

pose7d=pose(1.8288,0.5048,-pi/2);
system.loadTrajectory('cubic', pose7d);
system.executeTrajectory(pose7d, p1Controller, delay, Vmax, 1, k1, useLaser);
robot.forksDown();
pause(0.5);
robot.sendVelocity(-0.25,-0.25);
pause(t6);   % make robot go back about 0.3m
robot.sendVelocity(0,0);

%
% % system = mrplSystem(robot, robotStPose, localizer);
% % system.encCurrentPose=robotStPose;
% % system.refCurrentPose=robotStPose;
% % pose1r=system.loadTrajectory('fl_rotate', [pi, Vmax*6, 1]);
% %
% % for i=1:3
% %     if useLaser
% %         [success, robotStPose] = localizer.initpos(robotEstPose);
% %     else
% %         success = 1;
% %         robotStPose = robotEstPose;
% %     end
% %     system = mrplSystem(robot, robotStPose, localizer);
% %     system.encCurrentPose=robotStPose;
% %     system.resetReference();
% %     pose11= system.loadTrajectory('rotate', [pi, Vmax*6, 1]);
% %     system.executeTrajectory(pose11, p3Controller, delay, Vmax, 1, k2, useLaser);
% %     display('second');
% %     %1,1
% %     % system.resetReference();
% %     pose1 = pose(0.3048+0.01+(i-1)*(0.3048-0.01), 0.54 , pi/2.0);
% %     system.loadTrajectory('cubic', pose1);
% %     system.executeTrajectory(pose1, p1Controller, delay, Vmax, 1, k, useLaser);
% %     display('third') ;
% %     %1.2
% %     system.resetReference();
% %     robotStPose=system.encCurrentPose;
% %     %object position relative to sensor
% %     ranges = neatoLaserRanges;
% %
% %     image = rangeImage(ranges, 1, 1, robotStPose);
% %     Tso = pose(image.middlex+0.03, image.middley, image.theta);
% %         display(Tso.x);
% %         display(Tso.y);
% %         display(Tso.th);
% %
% %     %homogeneous transforms from world to goal frame
% %     Twr = system.encCurrentPose;
% %     Trs = pose(robotModel.laser_l, 0, 0);
% %     Tog = pose(-robotModel.objOffset-robotModel.frontOffset, 0, 0);
% %     pose12 = pose(pose.matToPoseVec(Twr.bToA * Trs.bToA * Tso.bToA * Tog.bToA));
% %     system.loadTrajectory('cubic', pose12);
% %     system.executeTrajectory(pose12, p1Controller, delay, Vmax, 1, k1, useLaser);
% %     robot.forksUp();
% %     system.resetReference();
% %     pose13 = system.loadTrajectory('straight', [0.005, 0.2, -1]);
% %     system.executeTrajectory(pose13, p2Controller, delay, Vmax, 1, k, useLaser);
% %     system.resetReference();
% %     pose14= system.loadTrajectory('rotate', [pi, Vmax*6, 1]);
% %     system.executeTrajectory(pose14, p3Controller, delay, Vmax, 1, k2, useLaser);
% %     system.resetReference();
% %     pose2 = pose(0.5344+(i-1)*0.1524,0.3, -pi/2.0);
% %     system.loadTrajectory('cubic', pose2);
% %     system.executeTrajectory(pose2, p1Controller, delay, Vmax, 1, k2, useLaser);
% %     system.resetReference();
% %     robot.forksDown();
% %     pose14 = system.loadTrajectory('straight', [0.03, 0.2, -1]);
% %     system.executeTrajectory(pose14, p2Controller, delay, Vmax, 1, k, useLaser);
% %
% %     robotEstPose=system.encCurrentPose;
% % end
% % robot.stopLaser();

 






