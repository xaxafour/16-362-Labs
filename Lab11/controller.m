classdef controller < handle
    
    properties(Constant)
    end
    
    properties(Access = private)
    end
    
    properties(Access = public)
        kpx
        kpy
        kpth
    end
    
    methods(Static = true)
        
        function [xerr, yerr] = computeError(x, xref, y, yref, th)
            error(:, 1) = ([cos(th), -sin(th); sin(th), cos(th)])\[xref-x; yref-y];
            xerr = error(1, 1);
            yerr = error(2, 1);

        end
        
    end
    
    methods(Access = private)
    end
            
    methods(Access = public)
        
        function obj = controller(kpx, kpy,kpth)
            obj.kpx = kpx;
            obj.kpy = kpy;
            obj.kpth = kpth;
        end
        
        function [uv, uw] = computeControl(obj, v, w, x, xref, y, yref, th,thref)
            error(:, 1) = ([cos(th), -sin(th); sin(th), cos(th)])\[xref-x; yref-y];

                uv = v + obj.kpx * error(1, 1);
                uw = w + obj.kpy * error(2, 1)+obj.kpth*atan2(sin(thref - th), cos(thref - th));
        end
        
    end
end