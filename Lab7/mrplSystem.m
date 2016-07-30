classdef mrplSystem < handle
    
    properties(Constant)
    end
    
    properties(Access = private)
    end
    
    properties(Access = public)
        robot
        follower
        refCurrentPose
        encCurrentPose
    end
    
    methods(Static = true)
    end
    
    methods(Access = private)
    end
    
    methods(Access = public)
        
        function obj = mrplSystem(robot, startPose)
            obj.robot = robot;
            if obj.robot ~= 0
                obj.robot.encoders.NewMessageFcn = @newEncoderDataCallback;
            end
            obj.refCurrentPose = startPose;
            obj.encCurrentPose = startPose;
        end
        
        function executeTrajectory(obj, targetPose, controller, delay, Vmax, figureNum)
            global encoderL0 encoderR0 encoderT0 encoderL encoderR encoderT i
            encoderL0 = 0;
            encoderR0 = 0;
            encoderT0 = 0;
            encoderL = 0;
            encoderR = 0;
            encoderT = 0;
            i = 0;
            %targetPose = pose(pose.matToPoseVec(obj.refCurrentPose.aToB * targetPose.bToA));
            
            %create trajectory and trajectoryFollower based on targetPose
            trajectory = cubicSpiral.planTrajectory(targetPose.x, targetPose.y, targetPose.th, 1);
            trajectory.planVelocities(Vmax);
            tf = trajectory.getTrajectoryDuration();
            obj.follower = trajectoryFollower(obj.robot, delay, trajectory, obj.refCurrentPose, obj.encCurrentPose, controller);
            
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
                    pause(1);
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
            %trajectory
            obj.refCurrentPose = pose(pose.matToPoseVec(obj.refCurrentPose.bToA*targetPose.bToA));
            obj.encCurrentPose = pose(obj.follower.x(obj.follower.i), obj.follower.y(obj.follower.i), obj.follower.th(obj.follower.i));
        end
        
    end
end