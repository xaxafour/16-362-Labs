classdef trajectoryFollower < handle
    
    properties(Constant)
        scale = 1.1;
    end
    
    properties(Access = private)
    end
    
    properties(Access = public)
        robot
        delay
        trajectory
        refStartPose
        encStartPose
        
        i
        t0
        t
        enct
        
        encL
        encR
        vDl
        vDr
        
        th
        x
        y
        
        thref
        xref
        yref
        
        controller
    end
    
    methods(Static = true)
    end
    
    methods(Access = private)
    end
            
    methods(Access = public)
        
        function obj = trajectoryFollower(robot, delay, trajectory, refStartPose, encStartPose, controller)
            obj.robot = robot;
            obj.delay = delay;
            obj.trajectory = trajectory;
            obj.refStartPose = refStartPose;
            obj.encStartPose = encStartPose;
            
            obj.i = 1;
            obj.t0 = 0;
            obj.t = zeros(1);
            obj.enct = zeros(1);
        
            obj.encL = zeros(1);
            obj.encR = zeros(1);
            obj.vDl = 0;
            obj.vDr = 0;
        
            obj.th(1) = encStartPose.th;
            obj.x(1) = encStartPose.x;
            obj.y(1) = encStartPose.y;
        
            obj.thref(1) = refStartPose.th;
            obj.xref(1) = refStartPose.x;
            obj.yref(1) = refStartPose.y;
            
            obj.controller = controller;
        end
		
		function t = currentTime(obj)
			t = obj.t(obj.i);
		end
        
        function sendVelocities(obj)
            global encoderL encoderR encoderT
            
            %set initial time
            if obj.i == 1
                c = clock;
                obj.t0 = c(5)*60 + c(6);
            end
            
            %compute current time, Dt based on system time, encDt based
            %on encoder timestamps
            obj.i = obj.i+1;
            c = clock;
            obj.t(obj.i) = c(5)*60 + c(6) - obj.t0;
            Dt = obj.t(obj.i) - obj.t(obj.i-1);
            obj.enct(obj.i) = encoderT;
            encDt = obj.enct(obj.i) - obj.enct(obj.i-1);
            
            %compute left and right wheel v based on encDt
            obj.encL(obj.i) = encoderL;
            obj.encR(obj.i) = encoderR;
            encDl = obj.encL(obj.i)-obj.encL(obj.i-1);
            encDr = obj.encR(obj.i)-obj.encR(obj.i-1);
            if encDt ~= 0
                obj.vDl = encDl/encDt;
                obj.vDr = encDr/encDt;
            end
            
            %compute current th, x, y by integrating wheel velocities with
            %respect to Dt
            [Dv, Dw] = robotModel.vlvrToVw(obj.vDl, obj.vDr);
            Dth = Dw*Dt;
            obj.th(obj.i) = obj.th(obj.i-1)+Dth;
            Ds = Dv*Dt;
            Dx = cos(obj.th(obj.i))*Ds;
            Dy = sin(obj.th(obj.i))*Ds;
            %scale Dx and Dy to estimate real position
            Dx = Dx * obj.scale;
            Dy = Dy * obj.scale;
            obj.x(obj.i) = obj.x(obj.i-1)+Dx; 
            obj.y(obj.i) = obj.y(obj.i-1)+Dy;

            %get the reference pose with delay at time t
            refPose = obj.trajectory.getPoseAtTime(obj.t(obj.i)-obj.delay);
            refPose = pose(refPose(1), refPose(2), refPose(3));
            %transform reference pose based on startPose
            p = pose.matToPoseVec(obj.refStartPose.bToA*refPose.bToA);
            obj.xref(obj.i) = p(1);
            obj.yref(obj.i) = p(2);
            obj.thref(obj.i) = p(3);
            %get the reference v and w at time t
            v = obj.trajectory.getVAtTime(obj.t(obj.i));
            w = obj.trajectory.getwAtTime(obj.t(obj.i));
            
            %compute controlled v and w
            [uv, uw] = obj.controller.computeControl(v, w, obj.x(obj.i), obj.xref(obj.i), obj.y(obj.i), obj.yref(obj.i), obj.th(obj.i));
            
            %send vl and vr commands to robot
            [vl, vr] = robotModel.VwTovlvr(uv, uw);
            [vl, vr] = robotModel.limitWheelVelocities([vl, vr]);
            if obj.robot ~= 0
                obj.robot.sendVelocity(vl, vr);  
            end
        end

    end
end