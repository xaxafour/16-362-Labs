classdef mrplSystem < handle
    
    properties(Constant)
    end
    
    properties(Access = private)
    end
    
    properties(Access = public)
        robot
        follower
    end
    
    methods(Static = true)
    end
    
    methods(Access = private)
    end
    
    methods(Access = public)
        
        function obj = mrplSystem(robot)
            obj.robot = robot;
            obj.robot.encoders.NewMessageFcn = @newEncoderDataCallback;
        end
        
        function executeTrajectory(obj, trajectory, controller, delay, Vmax, figureNum)
            global encoderL0 encoderR0 encoderT0 encoderL encoderR encoderT i
            encoderL0 = 0;
            encoderR0 = 0;
            encoderT0 = 0;
            encoderL = 0;
            encoderR = 0;
            encoderT = 0;
            i = 0;

            trajectory.planVelocities(Vmax);
            tf = trajectory.getTrajectoryDuration();
            obj.follower = trajectoryFollower(obj.robot, delay, trajectory, controller);
            
            figure(figureNum);
            myPlot1 = plot(obj.follower.xref, obj.follower.yref);
            hold on;
            myPlot2 = plot(obj.follower.x, obj.follower.y);
            xlabel('x(m)');
            ylabel('y(m)');
            legend('reference', 'encoder');
            
            loops = 0;
            while obj.follower.currentTime() < tf
                if loops < 2
                    obj.robot.sendVelocity(0, 0);
                elseif loops == 2  
                    i = 1;
                    pause(1);
                else
                    obj.follower.sendVelocities();
                    set(myPlot1, 'xdata', [get(myPlot1, 'xdata') obj.follower.xref(obj.follower.i)], 'ydata', [get(myPlot1, 'ydata') obj.follower.yref(obj.follower.i)]);
                    set(myPlot2, 'xdata', [get(myPlot2, 'xdata') obj.follower.x(obj.follower.i)], 'ydata', [get(myPlot2, 'ydata') obj.follower.y(obj.follower.i)]);
                    drawnow;
                    i = i+1;
                end
                loops = loops+1;
                
                pause(0.1);
            end
            obj.robot.sendVelocity(0, 0);
        end
        
    end
end