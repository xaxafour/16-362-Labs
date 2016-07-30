classdef straightTrajectory < handle
    
    properties(Constant)
        
    end
    
    properties(Access = private)
    end
    
    properties(Access = public)
        dist = 0;
        vmax = 0;
        sign = 0;
        amax = 0.25;
        numSamples = 201;
        timeArray = [];
        poseArray = [];
        VArray = [];
        wArray = [];
    end
    
    methods(Static = true)
    end
    
    methods(Access = private)
        
        function integrateCommands(obj)
            len = obj.numSamples;
            obj.poseArray  = zeros(3,len);
            
            % Place robot in initial state
            obj.poseArray(1,1) = 0.0;
            obj.poseArray(2,1) = 0.0;
            obj.poseArray(3,1) = 0.0;
            
            dt = obj.getTrajectoryDuration() / obj.numSamples;
            th = 0;
            
            for i=1:obj.numSamples-1
                obj.timeArray(i) = (i-1)*dt;
                V = obj.getVAtTime(obj.timeArray(i));
                w = obj.getwAtTime(obj.timeArray(i));
                ds = V*dt;
                dth = w*dt;
                th = th + dth;
                obj.poseArray(1, i+1) = obj.poseArray(1, i) + cos(th)*ds;
                obj.poseArray(2, i+1) = obj.poseArray(2, i) + sin(th)*ds;
                obj.poseArray(3, i+1) = th;
            end
            obj.timeArray(i+1) = obj.timeArray(i) + dt;
            
        end
        
    end
    
    methods(Access = public)
        
        function obj = straightTrajectory(dist, vmax, sign)
            obj.dist = dist;
            obj.vmax = vmax;
            obj.sign = sign;
            obj.integrateCommands();
        end 
        
        function planVelocities(obj,Vmax)
        end
        
        function time  = getTrajectoryDuration(obj)
            time  = (abs(obj.dist)/obj.vmax + obj.vmax/obj.amax);
        end
        
        function V  = getVAtTime(obj,t)
            tf = obj.getTrajectoryDuration();
            tramp = obj.vmax / obj.amax;
            
            if t < 0
                V = 0;
            elseif t < tramp
                V = obj.sign * (obj.amax*t);
            elseif t > tramp && t < tf-tramp
                V = obj.sign * obj.vmax;
            elseif t > (tf - tramp) && t < tf
                V = obj.sign *(obj.vmax - obj.amax * (t-tf+tramp));
            else
                V = 0;
            end
        end
        
        function w  = getwAtTime(obj,t)
            w = 0;
        end
        
        function pose  = getPoseAtTime(obj,t)
            x = interp1(obj.timeArray,obj.poseArray(1,:),t,'pchip','extrap');
            y = interp1(obj.timeArray,obj.poseArray(2,:),t,'pchip','extrap');
            th = interp1(obj.timeArray,obj.poseArray(3,:),t,'pchip','extrap');
            pose  = [x ; y ; th];
        end
        
    end
end