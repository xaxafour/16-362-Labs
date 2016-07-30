classdef figure8ReferenceControl < handle
    
    properties(Constant)
    end
    
    properties(Access = private)
        Ks
        Kv
        tPause
        tf
    end
    
    properties(Access = public)
    end
    
    methods(Static = true)
    end
    
    methods(Access = private)
    end
            
    methods(Access = public)
        
        function obj = figure8ReferenceControl(Ks,Kv,tPause)
        % Construct a figure 8 trajectory. It will not start until
        % tPause has elapsed and it will stay at zero for tPause
        % afterwards. Kv scales velocity up when > 1 and Ks scales
        % the size of the curve itself down. 0.4 is a good value
        % for both.
            obj.Ks = Ks;
            obj.Kv = Kv;
            obj.tPause = tPause;
            obj.tf = 12.565/Kv*Ks;
        end
        
        function [v, w] = computeControl(obj,timeNow)
        % Return the linear and angular velocity that the robot
        % should be executing at time timeNow. Any zero velocity
        % pauses specified in the constructor are implemented here
        % too.
            t = timeNow - obj.tPause;
            if (t < 0) || (t > obj.tf)
                vl = 0;
                vr = 0;
                [v , w] = robotModel.vlvrToVw(vl, vr);
%             elseif (t+2 < 0) || (t-2 > obj.tf)
%                 vr = ((0.3*obj.Kv) + 0.14125*(obj.Kv/obj.Ks) * sin((t*obj.Kv)/(2*obj.Ks)))/2;
%                 vl = ((0.3*obj.Kv) - 0.14125*(obj.Kv/obj.Ks) * sin((t*obj.Kv)/(2*obj.Ks)))/2;
%                 [v , w] = robotModel.vlvrToVw(vl, vr);
            else
                vr = (0.3*obj.Kv) + 0.14125*(obj.Kv/obj.Ks) * sin((t*obj.Kv)/(2*obj.Ks));
                vl = (0.3*obj.Kv) - 0.14125*(obj.Kv/obj.Ks) * sin((t*obj.Kv)/(2*obj.Ks));
                [v , w] = robotModel.vlvrToVw(vl, vr);
            end
        end
        
        function duration = getTrajectoryDuration(obj)
        % Return the total time required for motion and for the
        % initial and terminal pauses.
            duration = obj.tPause + obj.tf + obj.tPause;
        end
        
    end
end