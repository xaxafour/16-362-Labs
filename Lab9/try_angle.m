%ob1 =  lineObject;
%ob1.lines=[0.5 0.5; 0.5 -0.5; -0.5 -0.5; -0.5 0.5; 0.5 0.5];
% ob2 =  lineObject;
% ob2.lines = [-0.756 1.044;-0.844 0.956;-0.756 1.044];
ob4 = lineObject;
ob4.lines=[-2 2;-2 -2;2 -2;-2 -2; -2 2];

%ob3 =  lineObject;
%ob3.lines = [1 0; 0.9116 0.0884;1 0];
%obj = [ob1,ob2];
%map = lineMap(ob1);
%robot.map;
robot.genMap(ob4);
robot.map;
robot.startLaser();
pause(2);
a=robot.laser.LatestMessage.Ranges;