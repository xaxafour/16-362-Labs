classdef rangeImage < handle
    %rangeImage Stores a 1D range image and provides related services.
    properties(Constant)
        maxUsefulRange = 2.0;
        minUsefulRange = 0.05;
        maxRangeForTarget = 1.0;
        palletlength=0.125;
    end
    properties(Access = public)
        rArray = [];
        tArray = [];
        xArray = [];
        yArray = [];
        %the arrays only with useful points
        %newrArray = [];
        %newtArray = [];
        newxArray = [];
        newyArray = [];
        numPix;
        middlex =0;
        middley=0;
        theta;  % radian system
    end
    
    methods(Access = public)
        function obj = rangeImage(ranges,skip,cleanFlag)   % why we can decide skip?
            % Constructs a rangeImage for the supplied data.
            % Converts the data to rectangular coordinates
            if(nargin == 3)
                n=0;
                for i=1:skip:length(ranges)
                    n = n + 1;
                    obj.rArray(n) = ranges(i);
                    obj.tArray(n) = (i-1)*(pi/180);
                    obj.xArray(n) = ranges(i)*cos(obj.tArray(n));
                    obj.yArray(n) = ranges(i)*sin(obj.tArray(n));
                end
                obj.numPix = n;
                if cleanFlag 
                    obj.removeBadPoints();
                else
                    obj.getmodelPts();
                end
                
            end
        end
        
        function modelPts = getmodelPts(obj)
            obj.rArray(obj.rArray==0)=Inf;
            [~,index]=find(obj.rArray<=3);
%             k=diff([0 diff(A)==1 0]);
%             begin=find(k==1);
%             End=find(k==-1);
%             number= begin-End;
            a=length(index);
            modelPts=zeros(3,25);
            scale=floor(a/25);
            for i=1:25
%                 if i<a-1
%                    if index(i+1)-index(i)>2
%                        continue;
%                    else
                     modelPts(1,i)=obj.xArray(index(1+(i-1)*scale));
                     modelPts(2,i)=obj.yArray(index(1+(i-1)*scale));
                     modelPts(3,i)=1;
            end
      end
           
        
        
        function [positivemin,index]=getfirstnum(obj)
             %find out the nearest point
            obj.rArray(obj.rArray==0)=Inf;
            positivemin=min(obj.rArray);
            [~,index]=find(obj.rArray==positivemin);
            index = index(1);
            %make sure that the positivemin is legal
            while positivemin>obj.maxUsefulRange || positivemin < obj.minUsefulRange
                obj.rArray(index)= NaN;
                positivemin=min(data);
                [~,index]=find(data==positivemin);
            end
        end
        
        function removeBadPoints(obj)
            %find out the nearest point
            [positivemin,index]=obj.getfirstnum();
            
            %decide the point robot need to check on each side to find out a line based on the
            %distance between pallet and robot
            pointnum = ceil(atan(obj.palletlength/positivemin)/pi*180+1);
            %obj.newrArray(1) = positivemin;
            %obj.newtArray(1) = obj.tArray(index);
            obj.newxArray(1) = obj.xArray(index);
            obj.newyArray(1) = obj.yArray(index);
            badflagminus=0;  % used to record the condition that need to stop(if 2 sequent points are bad points)  (when calculate index-i)
            badflagplus=0;   %used to record the condition that need to stop (if 2 sequent points are bad points) (when calculate index+i)
            totallength=0;   %the length between 2 supposed end points
            error=0;%leave a little space for error
            indexminus=index;
            indexadd=index;
            n=1;
            for i = 1:pointnum
                %use valid condition to decide whether a point canbe used
                %condition1: the difference between 2 r < obj.palletlength+error(some error is permited)
                %condition2: the length between point1(x,y)and point2(x,y)< obj.palletlength+error(some error is permited)
                %condition3: the length between 2 end points of the line <
                %obj.palletlength+error && > obj.palletlength-error
                indexadd=obj.inc(indexadd);
                
                length=((obj.xArray(indexadd)-obj.newxArray(1))^2+(obj.yArray(indexadd)-obj.newyArray(1))^2)^0.5;
                validcondition1 = (length<= obj.palletlength+error)&& (obj.rArray(indexadd)<obj.rArray(index)+obj.palletlength+error);
                if badflagplus<2
                    if validcondition1
                        obj.newxArray = [obj.newxArray,obj.xArray(indexadd)];
                        obj.newyArray = [obj.newyArray,obj.yArray(indexadd)];
                        n=n+1;
                    else
                        
                        badflagplus= badflagplus+1;
                    end
                end
                indexminus=obj.dec(indexminus);
                length=((obj.xArray(indexminus)-obj.newxArray(end))^2+(obj.yArray(indexminus)-obj.newyArray(end))^2)^0.5;
                validcondition2 = (length<= obj.palletlength+error)&& (obj.rArray(indexminus)<obj.rArray(index)+obj.palletlength+error);
                if badflagminus<2
                    if validcondition2
                        obj.newxArray = [obj.xArray(indexminus),obj.newxArray];
                        obj.newyArray = [obj.yArray(indexminus),obj.newyArray];
                        n=n+1;
                    else
                        badflagminus= badflagminus+1;
                    end
                end
                %totallength = ((obj.newxArray(1)-obj.newxArray(end))^2+(obj.newyArray(1)-obj.newyArray(end))^2)^0.5;
                %if totallength >= obj.palletlength %the threshold can be a little more than obj.palletlength. It depends on the error of the sensor(I think it is always a little bigger or the same)
                %   break;
                %end
            end
            %length=length(obj.newxArray);
            if n==1  %if from robot's sight, the pallet is a point, we need to make up another point
                obj.newxArray(2)= (obj.rArray(index)+obj.palletlength)*cos(obj.tArray(index));
                obj.newyArray(2)= (obj.rArray(index)+obj.palletlength)*sin(obj.tArray(index));
            end
            line = polyfit(obj.newxArray,obj.newyArray,1); %get the line
            if line(1)==Inf || line(1)==-Inf ||line(2)==Inf || line(2)==-Inf % the condition of x=constant
                line=polyfit(obj.newyArray,obj.newxArray,1);
                range1x=line(2);
                range2x=line(2);
                range1y=obj.newyArray(1);
                range2y=obj.newyArray(end);
                if range1x<0
                    obj.theta = pi;
                else
                    obj.theta = 0;
                end
                obj.middlex=(range1x+range2x)/2;
                obj.middley = (range1y+range2y)/2;
            else %calculate the edges of the line
                range1x=line(1)*(obj.newyArray(1)-line(2)+1/line(1)*obj.newxArray(1))/(1+line(1)^2);
                range1y=line(1)*(obj.newxArray(1)+line(2)/line(1)+line(1)*obj.newyArray(1))/(1+line(1)^2);
                range2x=line(1)*(obj.newyArray(end)-line(2)+1/line(1)*obj.newxArray(end))/(1+line(1)^2);
                range2y=line(1)*(obj.newxArray(end)+line(2)/line(1)+line(1)*obj.newyArray(end))/(1+line(1)^2);
                obj.theta=atan2(-range2x+range1x,range2y-range1y);
                obj.middlex=(range1x+range2x)/2;
                obj.middley = (range1y+range2y)/2;
                %                 q = atan2(obj.middley,obj.middlex);
%                 obj.theta=ltheta-pi/2;
%                 if ltheta>=-pi/2 && ltheta<=pi/2
%                     th1=ltheta+pi/2;
%                     th2=ltheta-pi/2;
%                 else if ltheta<=pi && ltheta > pi/2
%                         th1=ltheta-pi/2;
%                         th2=ltheta-1.5*pi;
%                     else
%                         th1=ltheta+pi/2;
%                         th2=1.5*pi+ltheta;
%                     end
%                 end
%                 if obj.middlex>=0
%                     if abs(th1)<abs(th2)
%                         obj.theta=th1;
%                     elseif abs(th1)==abs(th2)
%                             if obj.middley>0
%                                 obj.theta=pi/2;
%                             else
%                                 obj.theta=-pi/2;
%                                 obj.theta=pi/2;
% 
%                             end
%                     else
%                         obj.theta=th2; 
%                    end
%                     else
%                         if abs(th1)<abs(th2)
%                             obj.theta=th2;
%                         elseif abs(th1)==abs(th2)
%                             if obj.middley>0
%                                 obj.theta=pi/2;
%                             else
%                                 obj.theta=-pi/2;
%                             end
%                             
%                         else
%                             obj.theta=th1;
%                         end
%                  end
                    %                     r11 = r1T1.get(q,th1);
                    %                     r12=r1T1.get(q,th2);
                    %                     r21 = r2T1.get(q,th1);
                    %                     r22 = r2T1.get(1,th2);
                    
                    %                     if min([r11,r21])<=min([r12,r22])
                    %                         obj.theta=th1;
                    %                     else
                    %                         obj.theta=th2;
                    %                     end
                end
                
                a=[range1x,range2x];
                b=[range1y,range2y];
                figure(1);
                hold on;
                xlim([-1.5 1.5]);
                ylim([-1.5 1.5]);
                axis('square');
                plot(a,b);
                plot(0,0,'x');
                plot(obj.middlex,obj.middley,'x');
                angle = obj.theta*180/pi;
                display(angle);
                length=((range1x-range2x)^2+(range1y-range2y)^2)^0.5;
                display(length);
                %     obj.x1=range1x;
                %     obj.x2=range2x;
                %     obj.y1=range1y;
                %     obj.y2=range2y;
                % if the distance of pallet and robot is 0.05m, the robot should check
                % takes all points above and below two range thresholds
                % out of the arrays. This is a convenience but the result % should not be used by any routine that expects the points
                % to be equally separated in angle. The operation is done
                % inline and removed data is deleted.
                
                
            end


    




function plotRvsTh(obj, maxRange)
% plot the range image after removing all points exceeding
% maxRange

end
function plotXvsY(obj, maxRange)
% plot the range image after removing all points exceeding % maxRange

end
function [err num th] = findLineCandidate(obj,middle,maxLen)
    
% Find the longest sequence of pixels centered at pixel
% “middle” whose endpoints are separated by a length less
% than the provided maximum. Return the line fit error, the
% number of pixels participating, and the angle of
% the line relative to the sensor.


end
function num = numPixels(obj)
num = obj.numPix;
end
% Modulo arithmetic on nonnegative integers. MATLABs choice to
% have matrix indices start at 1 has implications for
% calculations relating to the position of a number in the
% matrix. In particular, if you want an operation defined on
% the sequence of numbers 1 2 3 4 that wraps around, the
% traditional modulus operations will not work correctly.
% The trick is to convert the index to 0 1 2 3 4, do the
% math, and convert back.
function out = inc(obj,in)
% increment with wraparound over natural numbers
out = indexAdd(obj,in,1);
end
function out = dec(obj,in)
% decrement with wraparound over natural numbers
out = indexAdd(obj,in,-1);
end
function out = indexAdd(obj,a,b)
% add with wraparound over natural numbers. First number
% “a” is "natural" meaning it >=1. Second number is signed.
% Convert a to 0:3 and add b (which is already 0:3).
% Convert the result back by adding 1.
out = mod((a-1)+b,obj.numPix)+1;
end
end
end