classdef lineMapLocalizer < handle
    %mapLocalizer A class to match a range scan against a map in
    % order to find the true location of the range scan relative to
    % the map.
    properties(Constant)
        %maxErr = 0.05; % 5 cm
        maxErr = 0.5;
        minPts = 5; % min # of points that must match
    end
    properties(Access = private)
    end
    
    properties(Access = public)
        lines_p1 = [];
        lines_p2 = [];
        gain = 0.0;
        errThresh = 0.0;
        gradThresh = 0.0;
        J=zeros(1,3);
        robot;
    end
    
    methods(Access = public)
        function obj = lineMapLocalizer(robot, lines_p1,lines_p2,gain,errThresh,gradThresh)
            % create a lineMapLocalizer
            obj.lines_p1 = lines_p1;
            obj.lines_p2 = lines_p2;
            obj.gain = gain;
            obj.errThresh = errThresh;
            obj.gradThresh = gradThresh;
            obj.robot = robot;
        end
        
        function [success,outPose] = initpos(obj,robotEstPose)
                 
            ranges = obj.robot.laser.LatestMessage.Ranges;
            image = rangeImage(ranges, 1, 0);
            modelPts = image.getmodelPts();
            [success, outPose]=obj.refinePose(robotEstPose,modelPts,500);
        end
    
        function ro2 = closestSquaredDistanceToLines(obj,pi)
            % Find the squared shortest distance from pi to any line
            % segment in the supplied list of line segments.
            % pi is an array of 2d points
            % throw away homogenous flag
            pi = pi(1:2,:);
            r2Array = zeros(size(obj.lines_p1,2),size(pi,2));
            for i = 1:size(obj.lines_p1,2)
                [r2Array(i,:) , ~] = closestPointOnLineSegment(pi,...
                    obj.lines_p1(:,i),obj.lines_p2(:,i));
            end
            ro2 = min(r2Array,[],1);
        end
        
        function ids = throwOutliers(obj,pose,ptsInModelFrame)
            % Find index of outliers in a scan.
            worldPts = pose.bToA()*ptsInModelFrame;
            r2 = obj.closestSquaredDistanceToLines(worldPts);
            ids = find(sqrt(r2) > obj.maxErr);
        end
        
        function avgErr = fitError(obj,pose,ptsInModelFrame)
            % Find the standard deviation of perpendicular distances of
            % all points to all lines
            % transform the points
            
            % ptsInModelFrame is a series of (x,y)  and extra 1; pose is [x,y,th]
            worldPts = (pose.bToA())*ptsInModelFrame;
            r2 = obj.closestSquaredDistanceToLines(worldPts);
            % delete the points whose values are Inf
            r2(r2 == Inf) = [];
            err = sum(r2);
            num = length(r2);
            if(num >= lineMapLocalizer.minPts)
                avgErr = sqrt(err)/num;
            else
                % not enough points to make a guess
                avgErr = inf;
            end
        end
        
        function errPlus0 = getJacobian(obj,poseIn,modelPts)
            % Computes the gradient of the error function
            errPlus0 = obj.fitError(poseIn,modelPts);
            eps = 0.001;
            dp = [eps , 0.0 , 0.0];
            newPose=poseIn.plus(dp);
            newPose=pose(newPose);
            errPlus1 = obj.fitError(newPose,modelPts);
            obj.J(1) = (errPlus1-errPlus0)/eps;
            dp = [0.0 ; eps ; 0.0];
            newPose=poseIn.plus(dp);
            newPose=pose(newPose);
            errPlus1 = obj.fitError(newPose,modelPts);
            obj.J(2) = (errPlus1-errPlus0)/eps;
            dp = [ 0.0; 0.0 ; eps];
            newPose=poseIn.plus(dp);
            newPose=pose(newPose);
            errPlus1 = obj.fitError(newPose,modelPts);
            obj.J(3) = (errPlus1-errPlus0)/eps;
            
        end
        
        function [success, outPose]= refinePose(obj,inPose,ptsInModelFrame,maxIters)
            % refine robot pose in world (inPose) based on lidar
            % registration. Terminates if maxIters iterations is
            % exceeded or if insufficient points match the lines.
            % Even if the minimum is not found, outPose will contain
            % any changes that reduced the fit error. Pose changes that
            % increase fit error are not included and termination
            % occurs thereafter.
            
            success=1;
            
            inPose = pose(robotModel.senToWorld(inPose));
            ids = obj.throwOutliers(inPose,ptsInModelFrame);
            ptsInModelFrame(:,ids) = [];
            if isempty(ptsInModelFrame)
                success = 0;
                display('ptsInModelFrame is empty');
            end
            errPlus0 = obj.getJacobian(inPose,ptsInModelFrame);
            
            % get the magnitude of the gradient
            gradJ = sqrt(sum((obj.J).*(obj.J)));
            
            n=0;
            outPose=inPose;
            while errPlus0 > obj.errThresh && gradJ >obj.gradThresh
                
                dp=-1*obj.J*obj.gain;
                outPose = outPose.plus(dp);
                outPose=pose(outPose);
                errPlus0 = obj.getJacobian(outPose,ptsInModelFrame);
                n=n+1;
                
                if n>= maxIters
                    success=0;
                    break;
                end
            end
            outPose = pose(robotModel.robToWorld(outPose));
        end
    end
end


