% objLen = 0.125;
% len = objLen*0.0254;
% dx = 0.25;
% p4 = [dx+0.5-len/2.0; 1.25];
% p5 = [dx+0.5+len/2.0 ; 1.25];
% oob = lineObject;
% oob.lines = [p4' ; p5'];
% lob = lineObject;
% lob.lines = [0 1.22; 0 0; 1.22 0; 0 0; 0 1.22];
% map_objects = [lob oob]; % lob is the array of map lines
% %Generate the map with the objects in it.
% robot.genMap( map_objects );
% objPose = [(p4+p5)/2 ; -pi()/2.0];

% wall = lineObject;
% wall.lines = [-0.5 0.72; -0.5 -0.5 ;0.72 -0.5 ; -0.5 -0.5; -0.5 0.72];
% obj = lineObject;
% obj.lines=[0.8 0.4;0.7116 0.4884;0.8 0.4];
% map_objects = [wall obj];
% robot.genMap( map_objects );

% 
% wall = lineObject;
% wall.lines = [-0.5 0.72; -0.5 -0.5 ;0.72 -0.5 ; -0.5 -0.5; -0.5 0.72];
% obj = lineObject;
% obj.lines=[0.8 0;0.8884 -0.0884;0.8 0];
% 
% objn = lineObject;
% objn.lines=[0 0.8;0.5 0.8;0 0.8];
% map_objects = [wall obj objn];
% robot.genMap( map_objects );
% 
robotEstPose = pose(0.89, 1.58, pi/2);
robot.startLaser();
pause(5);
ranges = robot.laser.LatestMessage.Ranges;
image = rangeImage(ranges, 1, 1,robotEstPose);
modelPts = image.getmodelPts();
localizer = lineMapLocalizer(robot, lines_p1,lines_p2,0.1,0.001,0.0005);
[success, outPose]=localizer.refinePose(robotEstPose,modelPts,20);
display(outPose.x);
display(outPose.y);
display(outPose.th);
robot.stopLaser();
