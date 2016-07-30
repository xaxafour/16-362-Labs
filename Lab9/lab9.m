%robot.startLaser();
%pause(5);
lines_p1=[0,1.22; 0,0];
lines_p2=[0,0; 1.22,0];
global fh;
fh = figure(1);
hold on;
vGain=1;
driver = robotKeypressDriver(fh);
localizer = lineMapLocalizer(lines_p1,lines_p2,0.02,0.001,0.0005);
robotPose = pose(15*0.0254,9*0.0254,pi()/2.0);

% j = 1;
% outPoseLog = zeros(1);
% modelPtsLog = zeros(1);

tic;
while toc < 70
    %sample the laser for useful points
    ranges = robot.laser.LatestMessage.Ranges;
    image = rangeImage(ranges, 1, 0);
    modelPts = image.getmodelPts();
    
    %create a pose estimate
    [success, outPose]=localizer.refinePose(robotPose,modelPts,25);
    %display(success);
    robotPose = outPose;
    
    %plot walls, robot pose estimate, and model points
    robotDrawing = robotPose.bToA()*robotModel.bodyGraph();
    ids = localizer.throwOutliers(robotPose, modelPts);
    modelPts(:,ids) = [];
    worldLidarPts = robotModel.senToWorld(robotPose)*modelPts;
    clf;
    fh = plot(lines_p1(:,1), lines_p1(:,2), 'b', lines_p2(:,1), lines_p2(:,2), 'b', -0.1, -0.1);
    hold on;
    fh = plot(robotDrawing(1,:), robotDrawing(2,:), 'g');
    hold on;
    fh = scatter(worldLidarPts(1,:), worldLidarPts(2,:), 'r');
    drawnow;
    
    driver.drive(robot, vGain);
    
    %outPoseLog(j,1:3) = [outPose.x, outPose.y, outPose.th];
    %modelPtsLog(2*j-1:2*j,1:25) = modelPts(1:2,:);
    %j = j+1;
    pause(0.2);
end

robot.stopLaser();