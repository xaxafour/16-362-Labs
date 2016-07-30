classdef mrplSystem < handle
    
    properties(Constant)
    end
    
    properties(Access = private)
    end
    
    properties(Access = public)
        robot
        trajectory
        follower
        refCurrentPose
        encCurrentPose
        k
        localizer
    end
    
    methods(Static = true)
    end
    
    methods(Access = private)
    end
    
    methods(Access = public)
        
        function obj = mrplSystem(robot, startPose, localizer)
            obj.robot = robot;
            %if obj.robot ~= 0
                %obj.robot.encoders.NewMessageFcn = @newEncoderDataCallback;
                %obj.robot.laser.NewMessageFcn = @neatoLaserEventListener;
            %end
            obj.refCurrentPose = startPose;
            obj.encCurrentPose = startPose;
            obj.localizer = localizer;
        end
        
        function RobPose=s_str_rot(obj,t3,t4,t5,times)
            obj.robot.sendVelocity(-0.25,-0.25);
            pause(t3);
            obj.robot.sendVelocity(0,0);
%             obj.robot.sendVelocity(0.28,-0.28);
%             pause(t4);
%             obj.robot.sendVelocity(0.28,-0.28);
%             pause(t5);
            %obj.robot.sendVelocity(0,0); 
            robotEstPose = pose(0.3048*times,1.6288,pi/2);
            [~, robotStPose] = obj.localizer.initpos(robotEstPose);
            obj.refCurrentPose=robotStPose;
            obj.encCurrentPose=robotStPose;
            RobPose = robotStPose;
        end
        
        function resetReference(obj)
            obj.refCurrentPose = obj.encCurrentPose;
        end
        
        function worldPose = loadTrajectory(obj, name, parameters)
            if isequal(name, 'cubic')
                %parameters: pose
                x = parameters(1).x;
                y = parameters(1).y;
                th = parameters(1).th;
                targetPose = pose(x, y, th);
                targetPose = pose(pose.matToPoseVec(obj.refCurrentPose.aToB * targetPose.bToA));
                obj.trajectory = cubicSpiral.planTrajectory(targetPose.x, targetPose.y, targetPose.th, 1);
                worldPose = pose(x, y, th);
                
            elseif isequal(name, 'straight')
                %parameters: dist, vmax, sign
                dist = parameters(1);
                vmax = parameters(2);
                sign = parameters(3);
                obj.trajectory = straightTrajectory(dist, vmax, sign);
                relPose = pose(dist * sign, 0, 0);
                worldPose = pose(pose.matToPoseVec(obj.refCurrentPose.bToA * relPose.bToA));
                
                
                
                elseif isequal(name, 'fl_rotate')
                %parameters: angle, wmax, sign
                angle = parameters(1);
                wmax = parameters(2);
                sign = parameters(3);
                obj.trajectory = rotateTrajectory(angle, wmax, sign);
                relPose = pose(0, 0, angle * sign);
                worldPose = pose(pose.matToPoseVec(obj.refCurrentPose.bToA * relPose.bToA));
                
            end
            
%             elseif isequal(name, 'fl_rotate')
% %               parameters: angle, wmax, sign
%                 angle = parameters(1);
%                 wmax = parameters(2);
%                 sign = parameters(3);
% %                 Rangle=-obj.encCurrentPose.th+angle;
%                  obj.trajectory = rotateTrajectory(angle, wmax, sign);
%                 relPose = pose(0, 0, angle * sign);
%                 worldPose = pose(pose.matToPoseVec(obj.refCurrentPose.bToA * relPose.bToA));
% %                 worldPose=pose(obj.refCurrentPose.x,obj.refCurrentPose.y,angle);
%                 
%             end
            
        end
        
         function executeTrajectory(obj, targetPose, controller, delay, Vmax, figureNum, k, useLaser)
            global encoderL0 encoderR0 encoderT0 encoderL encoderR encoderT i
            encoderL0 = 0;
            encoderR0 = 0;
            encoderT0 = 0;
            encoderL = 0;
            encoderR = 0;
            encoderT = 0;
            i = 0;
            obj.k = k;
            
            %transform the given target from world to robot frame
            targetPose = pose(pose.matToPoseVec(obj.refCurrentPose.aToB * targetPose.bToA));
                
            %create trajectoryFollower based on loaded trajectory
            obj.trajectory.planVelocities(Vmax);
            tf = obj.trajectory.getTrajectoryDuration();
            obj.follower = trajectoryFollower(obj.robot, delay, obj.trajectory, obj.refCurrentPose, obj.encCurrentPose, controller,obj.localizer,obj.k, useLaser);
            
            %initialize plot
            figure(figureNum);
            myPlot1 = plot(obj.follower.xref, obj.follower.yref);
            hold on;
            myPlot2 = plot(obj.follower.x, obj.follower.y);
            xlabel('x(m)');
            ylabel('y(m)');
            legend('reference', 'encoder');
            
            %main control loop
            loops = 0;
            while obj.follower.currentTime() < tf
                if loops < 2
                    %run through the loop
                    if obj.robot ~= 0
                        obj.robot.sendVelocity(0, 0);
                    end                   
                elseif loops == 2
                    %reset i which resets callback values
                    i = 1;
%                     encoderL0 = double(obj.robot.encoders.LatestMessage.Left) / 1000.0;
%                     encoderR0 = double(obj.robot.encoders.LatestMessage.Right) / 1000.0;
%                     encoderT0 = double(obj.robot.encoders.LatestMessage.Header.Stamp.Sec) + double(obj.robot.encoders.LatestMessage.Header.Stamp.Nsec) / 1000000000.0;
                    pause(0.1);
                else
                    %integrate encoders, apply control, send velocities
                    obj.follower.sendVelocities();
                    %plot encoders and reference trajectory
                    set(myPlot1, 'xdata', [get(myPlot1, 'xdata') obj.follower.xref(obj.follower.i)], 'ydata', [get(myPlot1, 'ydata') obj.follower.yref(obj.follower.i)]);
                    set(myPlot2, 'xdata', [get(myPlot2, 'xdata') obj.follower.x(obj.follower.i)], 'ydata', [get(myPlot2, 'ydata') obj.follower.y(obj.follower.i)]);
                    drawnow;
                    i = i+1;
                end
                loops = loops+1;
                
                pause(0.1);
            end
            
            %stop the robot
            if obj.robot ~= 0
                obj.robot.sendVelocity(0, 0);
            end
            
            %set the current reference and encoder positions for the next
            %trajectory. To plan trajectory from the present position, make
            %reference trajectory equal to the fused one
            obj.refCurrentPose = pose(pose.matToPoseVec(obj.refCurrentPose.bToA*targetPose.bToA));
            obj.encCurrentPose = pose(obj.follower.x(obj.follower.i), obj.follower.y(obj.follower.i), obj.follower.th(obj.follower.i));
            [success, robotStPose] = obj.localizer.initpos(obj.encCurrentPose);
            if success
              obj.encCurrentPose = robotStPose;
            end
            obj.refCurrentPose=obj.encCurrentPose;
    end
    end
end