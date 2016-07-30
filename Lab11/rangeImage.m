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
%         wallxArray = [];
%         wallyArray=[];
        %the arrays only with useful points
        newxArray = [];
         newyArray = [];
         objcell = {};
%         objxArray = [];
%         objyArray = [];
        numPix;
        middlex =0;
        middley=0;
        theta;  % radian system
        pose;
%         walledge1;
%         walledge2;
    end
    
    methods(Access = public)
        function obj = rangeImage(ranges,skip,cleanFlag, pose1)   
            % Constructs a rangeImage for the supplied data.
            % Converts the data to rectangular coordinates
            if(nargin == 4)
                   obj.pose = pose1;
%                 obj.pose = pose(robotModel.senToWorld(pose1));
%                 wpts1 = obj.pose.aToB() *[0;1.22;1];
%                 wpts2 = obj.pose.aToB() *[1.22;0;1];
%                 wtheta1 = atan2(wpts1(2),wpts1(1));
%                 wtheta2 = atan2(wpts2(2),wpts2(1));
%                 if wtheta1< 0
%                     wtheta1=wtheta1+2*pi;
%                 end
%                 if wtheta2 < 0
%                    wtheta2=wtheta2+2*pi;
%                 end
%                 obj.walledge1 = floor(wtheta1*180/pi);
%                 obj.walledge2 = floor(wtheta2*180/pi);
%                 if obj.walledge2 > obj.walledge1
%                     pass= obj.walledge1;
%                     obj.walledge1=obj.walledge2;
%                     obj.walledge2 = pass;
%                 end

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
                    obj. findObject();
%                 else
%                     obj.getmodelPts();
                end
                
            end
        end
        
        
        function modelPts = getmodelPts(obj)
            obj.rArray(obj.rArray==0)=Inf;
            [~,index]=find(obj.rArray<=2);
%             k=diff([0 diff(A)==1 0]);
%             begin=find(k==1);
%             End=find(k==-1);
%             number= begin-End;
            a=length(index);
%             a= obj.walledge1 - obj.walledge2;
            modelPts=zeros(3,15);
            scale=floor(a/15);
            for i=1:15
%                 if i<a-1
%                    if index(i+1)-index(i)>2
%                        continue;
%                    else
                     modelPts(1,i)=obj.xArray(index(1+(i-1)*scale));
                     modelPts(2,i)=obj.yArray(index(1+(i-1)*scale));
                     modelPts(3,i)=1;
            end
      end
           
        
        
%         function [positivemin,index]=getfirstnum(obj)
%              %find out the nearest point
%             obj.rArray(obj.rArray==0)=Inf;
%             positivemin=min(obj.rArray);
%             [~,index]=find(obj.rArray==positivemin);
%             index = index(1);
%             %make sure that the positivemin is legal
%             while positivemin>obj.maxUsefulRange || positivemin < obj.minUsefulRange
%                 obj.rArray(index)= NaN;
%                 positivemin=min(data);
%                 [~,index]=find(data==positivemin);
%             end
%         end
        
function findObject(obj)
    m=0;
    error= 0.035;%leave a little space for error
    obj.rArray(obj.rArray==0)=Inf;
    obj.rArray(obj.rArray>2)=Inf; % limit the distance
    obj.rArray(1,16:344)=Inf;
    %find the point groups
    
    difference = diff(obj.rArray);  
    difference= [difference, obj.rArray(1)-obj.rArray(360)]; 
    boundary = find(abs(difference)>=0.04);  % find boundary
    l = max(size(boundary));
            % detect if the line is found, after we find one suitable line, we will not go on
    for i = 1:l%go through all boundary
        obj.newxArray=[];
        obj.newyArray=[];
        success=0;
        previndex = obj.dec(i,l);
        firstpt = obj.inc(boundary(previndex),360);   
        dif = boundary(i)-firstpt;      %number of points between two boundary
        if dif < 0
            dif=dif+360;
        end
        if dif>15 || dif<3    % if the munber is too small or too large, kick it out
            continue;
        end
        mid = ceil((boundary(i)+firstpt)/2);  % the middle point of the boundary
        if firstpt<=360 &&i==1
           mid = mod(mid+180, 360);
           if mid ==0 
               mid =1; 
           end
        end
     
        if obj.rArray(mid)>1.5     % delete the space bewteen two lines
            continue;
        end
        %decide the point robot need to check on each side to find out a line based on the
        %distance between pallet and robot
        pointnum = ceil(atan(obj.palletlength/2/obj.rArray(mid))/pi*180+1);
        %obj.newrArray(1) = positivemin;
        %obj.newtArray(1) = obj.tArray(index);
        obj.newxArray(1) = obj.xArray(mid);
        obj.newyArray(1) = obj.yArray(mid);
        badflagminus=0;  % used to record the condition that need to stop(if 2 sequent points are bad points)  (when calculate index-i)
        badflagplus=0;   %used to record the condition that need to stop (if 2 sequent points are bad points) (when calculate index+i)
        num = 0;
        indexminus=mid;
        indexadd=mid;
        %                 n=1;
        for k = 1:pointnum
            %use valid condition to decide whether a point canbe used
            %condition1: the difference between 2 r < obj.palletlength+error(some error is permited)
            %condition2: the length between point1(x,y)and point2(x,y)< obj.palletlength+error(some error is permited)
            %condition3: the length between 2 end points of the line <
            %obj.palletlength+error && > obj.palletlength-error
            indexadd=obj.inc(indexadd,obj.numPix);
            num=num+1;
            length=((obj.xArray(indexadd)-obj.newxArray(1))^2+(obj.yArray(indexadd)-obj.newyArray(1))^2)^0.5;
            validcondition1 = (length<= obj.palletlength+error)&&(length>= obj.palletlength-error);
            if badflagplus<2
                if (length<= obj.palletlength)
                      obj.newxArray = [obj.newxArray,obj.xArray(indexadd)];
                      obj.newyArray = [obj.newyArray,obj.yArray(indexadd)];

                 else
                
                       badflagplus= badflagplus+1;
                 end
                %                         n=n+1;

                
            end
           if validcondition1 && num>=dif-1
                 success=1;
                 break;
           end
           num=num+1;
            indexminus=obj.dec(indexminus,obj.numPix);
            length=((obj.xArray(indexminus)-obj.newxArray(end))^2+(obj.yArray(indexminus)-obj.newyArray(end))^2)^0.5;
            validcondition2 = (length<= (obj.palletlength+error))&& (length>=( obj.palletlength-error));

            if badflagminus<2
                if (length<= obj.palletlength)
                  obj.newxArray = [obj.xArray(indexminus),obj.newxArray];
                  obj.newyArray = [obj.yArray(indexminus),obj.newyArray];

                else
                  badflagminus= badflagminus+1;
                end
                %                         n=n+1;
                

            end
            if validcondition2 && num>=dif-1
                 success=1;
                 if num ==7
                     display('hah');
                 end
              break;
            end

        end

        if success==1
           m=m+1;
%            len = max(size(obj.newxArray));
%            a = ones(1,len);
           candidate = [obj.newxArray;obj.newyArray];
           obj.objcell{m} = candidate;
        end
    end
    %length=length(obj.newxArray);
    %             if n==1  %if from robot's sight, the pallet is a point, we need to make up another point
    %                 obj.newxArray(2)= (obj.rArray(index)+obj.palletlength)*cos(obj.tArray(index));
    %                 obj.newyArray(2)= (obj.rArray(index)+obj.palletlength)*sin(obj.tArray(index));
    %             end
    ansr=1;
    for i = 1:m
        if i+1 <=m
            if  obj.objcell{1,i}(1,1)<obj.objcell{1,i+1}(1,1)
                ansr = i;
            else
                ansr=i+1;
            end
        else 
            break;
        end
    end
    obj.newxArray = obj.objcell{1,ansr}(1,:);
    obj.newyArray = obj.objcell{1,ansr}(2,:);
         
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
    end
%     obj.middlex
%     obj.middley
%     obj.theta
    %                 a=[range1x,range2x];
    %                 b=[range1y,range2y];
    %                 figure(1);
    %                 hold on;
    %                 xlim([-1.5 1.5]);
    %                 ylim([-1.5 1.5]);
    %                 axis('square');
    %                 plot(a,b);
    %                 plot(0,0,'x');
    %                 plot(obj.middlex,obj.middley,'x');
    %                 angle = obj.theta*180/pi;
    %                 display(angle);
    %                 length=((range1x-range2x)^2+(range1y-range2y)^2)^0.5;
    %                 display(length);
    
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
function out = inc(obj,in,num)
% increment with wraparound over natural numbers
out = indexAdd(obj,in,num,1);
end
function out = dec(obj,in,num) % in is the index need to be increase or decrease; num is the total num of that array;
% decrement with wraparound over natural numbers
out = indexAdd(obj,in,num,-1);
end
function out = indexAdd(obj,a,num,b)
% add with wraparound over natural numbers. First number
% “a” is "natural" meaning it >=1. Second number is signed.
% Convert a to 0:3 and add b (which is already 0:3).
% Convert the result back by adding 1.
out = mod((a-1)+b,num)+1;
end
end
end