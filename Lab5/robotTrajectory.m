classdef robotTrajectory < handle
    
    properties(Constant)
    end
    
    properties(Access = private)
        referenceControl
        numSamples
    end
    
    properties(Access = public)
        t
        s
        v
        w
        p
    end
    
    methods(Static = true)        
    end
    
    methods(Access = private)
        
        function [t, s, v, w, p] = computeSamples(obj)
            n = obj.numSamples;
            dt = obj.referenceControl.getTrajectoryDuration() / n;
            
            t = zeros(n, 1);
            s = zeros(n, 1);
            v = zeros(n, 1);
            w = zeros(n, 1);
            p = zeros(n, 3);
            
            th = 0;
            x = 0;
            y = 0;
            p(1,:) = [x, y, th];
            s(1) = 0;
            for i = 1:(n-1)
                t(i) = (i-1)*dt;
                [v(i), w(i)] = obj.referenceControl.computeControl(t(i));
                ds = v(i)*dt;
                dth = w(i)*dt;
                th = th + dth;
                dx = cos(th)*ds;
                dy = sin(th)*ds;
                x = x + dx;
                y = y + dy;
                p(i+1,:) = [x, y, th];
                s(i+1) = s(i) + ds;
            end
            t(n) = n*dt;
            [v(n), w(n)] = obj.referenceControl.computeControl(t(n));
        end
        
    end
            
    methods(Access = public)
        
        function obj = robotTrajectory(referenceControl, numSamples)
            obj.referenceControl = referenceControl;
            obj.numSamples = numSamples;
            [obj.t, obj.s, obj.v, obj.w, obj.p] = computeSamples(obj);
        end
        
        function distance = getDistAtTime(obj, t)
            distance = interp1(obj.t, obj.s, t);
        end
        
        function linVel = getLinVelAtTime(obj, t)
            linVel = interp1(obj.t, obj.v, t);
        end
        
        function angVel = getAngVelAtTime(obj, t)
            angVel = interp1(obj.t, obj.w, t);
        end
        
        function pos = getPoseAtTime(obj, t)
            x = interp1(obj.t, obj.p(:,1), t);
            y = interp1(obj.t, obj.p(:,2), t);
            th = interp1(obj.t, obj.p(:,3), t);
            pos = pose(x, y, th);
        end

    end
end