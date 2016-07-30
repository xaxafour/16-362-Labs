classdef cubicSpiral < handle
    %cubicSpiral Implements a planar trajectory specified in terms of three
    % coefficients that adjust the terminal pose of the robot. The initial
    % pose is assumed to be the origin with zero curvature. The terminal
    % curvature is forced to be zero.
    
    properties(Constant)
        
    end
    
    properties(Access = private)
        parms = [0 0 1];
        sgn=1.0;
        rampLength = 0.05;
    end
    
    properties(Access = public)
        numSamples = 201;
        distArray = [];
        TArray = [];
        posArray = [];
        curvArray = [];
        vlArray = []
        vrArray = []
        VArray = [];
        wArray = [];
        ds2
    end
    
    methods(Access = public)
        function obj = cubicSpiral(parms,numSamples)
            obj.parms = parms;
            obj.ds2 = parms(3)/(numSamples-1);
            s2=0;
            t2 = 0;
            x2 = 0;
            y2 = 0;
            obj.posArray =zeros(3,numSamples);
            obj.distArray =zeros(1,numSamples);
            obj.curvArray  =zeros(1,numSamples);
            obj.distArray(1) = 0;
            obj.posArray(1,1)=0;
            obj.posArray(2,1)=0;
            obj.posArray(3,1)=0;
            for index = 1:numSamples
                s2 =(index-1)*obj.ds2;
                obj.curvArray(index)=s2*(parms(1)+parms(2)*s2)*(s2-parms(3));
                if index>1
                    obj.distArray(index)= s2;
                end
                t2 = t2+ obj.curvArray(index)*obj.ds2;
                % using the average of 2 points is more precise?
                %x2 = x2+ cos(t2)*obj.ds2;
                %y2 = y2+ sin(t2)*obj.ds2;
                %x2 = x2+ cos(t2-0.5*obj.curvArray(index)*obj.ds2)*obj.ds2;
                %y2 = y2+ sin(t2-0.5*obj.curvArray(index)*obj.ds2)*obj.ds2;
                x2 = x2+ cos(t2)*obj.ds2;
                y2 = y2+ sin(t2)*obj.ds2;
                
                if index >1
                    obj.posArray(1,index)=x2;
                    obj.posArray(2,index)=y2;
                    obj.posArray(3,index)=t2;
                end
            end
        end
        
        
        
        
        
        
        function planVelocities(obj,Vmax)
            % Plan the highest possible velocity for the path where no
            % wheel may exceed Vmax in absolute value.
            for i=1:obj.numSamples
                Vbase = Vmax;
                % Add velocity ramps for first and last 5 cm
                s = obj.distArray(i);
                sf = obj.distArray(obj.numSamples);
                if(abs(sf) > 2.0*obj.rampLength) % no ramp for short trajectories
                    sUp = abs(s);
                    sDn = abs(sf-s);
                    if(sUp < obj.rampLength) % ramp up
                        Vbase = Vbase * sUp/obj.rampLength;
                    elseif(sDn < 0.05) % ramp down
                        Vbase = Vbase * sDn/obj.rampLength;
                    end
                end
                % Now proceed with base velocity 
                %disp(Vbase);
                V = Vbase*obj.sgn; % Drive forward or backward as desired.
                K = obj.curvArray(i);
                w = K*V;
                vr = V + robotModel.W2*w;
                vl = V - robotModel.W2*w;               
                if(abs(vr) > Vbase)
                    vrNew = Vbase * sign(vr);
                    vl = vl * vrNew/vr;
                    vr = vrNew;
                end
                if(abs(vl) > Vbase)
                    vlNew = Vbase * sign(vl);
                    vr = vr * vlNew/vl;
                    vl = vlNew;
                end
                obj.vlArray(i) = vl;
                obj.vrArray(i) = vr;
                obj.VArray(i) = (vr + vl)/2.0;
                obj.wArray(i) = (vr - vl)/robotModel.W;                
            end
            % Now compute the times that are implied by the velocities and
            % the distances.
            obj.computeTimeSeries();
        end
        
 
        
        
        
        
        function planVelocities2(obj,Vmax)  % i think it is better to make v the same number with ds
            %a = 0.75
            obj.VArray =[];
            obj.wArray=[];
            sramp = Vmax^2/1.5;
            eth = floor(sramp/obj.ds2);
            
            if obj.distArray(201)<=2*sramp
                for e = 1 : 101
                    %v1 = (1.5*ds *e)^0.5;
                    %v2 = (1.5*ds*(e-1))^0.5;
                    obj.VArray(e) = (1.5*obj.ds2*(e-1))^0.5;
                    %w1 = v1*curvArray(e);
                    %w2 = v2*curvArray(e+1);
                    obj.wArray(e)=obj.VArray(e)*obj.curvArray(e);
                    [vl , vr] = robotModel.VwTovlvr(obj.VArray(e), obj.wArray(e));% function from robotmodel
                    absvmax = max(abs(vl),abs(vr));
                    if absvmax >= 0.29
                        vscale = 0.29/ absvmax;
                        vl = vl*vscale;
                        vr = vr*vscale;
                    end
                    [obj.VArray(e) ,obj.wArray(e)] = robotModel.vlvrToVw(vl, vr);
                end
                for e = 102:201
                    %v3 = (1.5*(201-e)*ds)^0.5;
                    %v4 = (1.5*(200-e)*ds)^0.5;
                    obj.VArray(e)=obj.VArray(202-e);
                    %w3 = v3*curvArray(e);
                    %w4 = v4*curvArray(e+1);
                    obj.wArray(e)=obj.VArray(e)*obj.curvArray(e);
                    [vl , vr] = robotModel.VwTovlvr(obj.VArray(e), obj.wArray(e));% function from robotmodel
                    absvmax = max(abs(vl),abs(vr));
                    if absvmax >= 0.29
                        vscale = 0.29/ absvmax;
                        vl = vl*vscale;
                        vr = vr*vscale;
                    end
                    [obj.VArray(e) ,obj.wArray(e)] = robotModel.vlvrToVw(vl, vr);
                end
            else
                for e = 1 : eth
                    obj.VArray(e)=(1.5*obj.ds2 *(e-1))^0.5;
                    obj.wArray(e)=obj.VArray(e)*obj.curvArray(e);
                    [vl , vr] = robotModel.VwTovlvr(obj.VArray(e), obj.wArray(e));% function from robotmodel
                    absvmax = max(abs(vl),abs(vr));
                    if absvmax >= 0.29
                        vscale = 0.29/ absvmax;
                        vl = vl*vscale;
                        vr = vr*vscale;
                    end
                    [obj.VArray(e) ,obj.wArray(e)] = robotModel.vlvrToVw(vl, vr);
                end
                for e = eth+1 : 201-eth
                    obj.VArray(e)=Vmax;
                    obj.wArray(e)=obj.VArray(e)*obj.curvArray(e);
                    [vl , vr] = robotModel.VwTovlvr(obj.VArray(e), obj.wArray(e));% function from robotmodel
                    absvmax = max(abs(vl),abs(vr));
                    if absvmax >= 0.29
                        vscale = 0.29/ absvmax;
                        vl = vl*vscale;
                        vr = vr*vscale;
                    end
                    [obj.VArray(e) ,obj.wArray(e)] = robotModel.vlvrToVw(vl, vr);
                end
                for e = 202-eth : 201
                    obj.VArray(e)= obj.VArray(202-e);
                    obj.wArray(e)=obj.VArray(e)*obj.curvArray(e);
                    [vl , vr] = robotModel.VwTovlvr(obj.VArray(e), obj.wArray(e));% function from robotmodel
                    absvmax = max(abs(vl),abs(vr));
                    if absvmax >= 0.29
                        vscale = 0.29/ absvmax;
                        vl = vl*vscale;
                        vr = vr*vscale;
                    end
                    [obj.VArray(e) ,obj.wArray(e)] = robotModel.vlvrToVw(vl, vr);
                end
                obj.computeTimeSeries();
            end
            % Plan the highest possible velocity for the path where no
            % wheel speed may exceed Vmax in absolute value.
        end
        
        
        
        function computeTimeSeries(obj)
            obj.TArray = zeros(1);
            for index1 = 1:200
                dt = (2* obj.ds2)/(obj.VArray(index1)+obj.VArray(index1+1));
                obj.TArray(index1+1) = obj.TArray(index1)+dt;
            end
        end
        
    end
    methods(Static = true)
        
        function makeLookupTable(scale)
            % Make 6 lookup tables to be used for cubicSpiral generation.
            % Principle is to sample densely in coefficient space and
            % integrate the curvature to find the curve. Then store the
            % result in a table that inverts the mapping.
            % On a 2.6 GHz core i7:
            % Scale = 100 takes 0.17 minutes
            % Scale = 50 takes 0.64 minutes
            % Scale = 10 takes 15 minutes
            % Scale = 2 takes 381 minutes
            % Scale = 1 takes > 125 hours (and still needs interpolation)
            
            startTic = tic();
            
            sMax = 1.0;   % length of curve
            qMax =  pi(); % max bearing
            qMin = -pi(); % min bearing
            tMax =  1.5*pi(); % max heading
            tMin = -1.5*pi(); % min heading
            
            numT = round(10*126/scale); % heading samples
            numQ = round(50*126/scale); % bearing samples
            
            % Do a quick performance estimate
            radius = 1.0;
            ds = radius*(qMax-qMin)/numQ;
            dt = (tMax-tMin)/numT;
            fprintf('Error predicted %f m in translation, %f rads in rotation\n',ds,dt);
            
            % Allocate tables
            a1Tab = scalarField(qMin,qMax,numQ,tMin,tMax,numT);
            b1Tab = scalarField(qMin,qMax,numQ,tMin,tMax,numT);
            r1Tab = scalarField(qMin,qMax,numQ,tMin,tMax,numT); % energy integral
            
            a2Tab = scalarField(qMin,qMax,numQ,tMin,tMax,numT);
            b2Tab = scalarField(qMin,qMax,numQ,tMin,tMax,numT);
            r2Tab = scalarField(qMin,qMax,numQ,tMin,tMax,numT); % energy integral
            
            clothSamples = 201;
            
            plotSamples = 10000;
            plotArrayX = zeros(plotSamples);
            plotArrayY = zeros(plotSamples);
            
            n = 1;
            aMax = 195.0/sMax/sMax;
            bMax = 440.0/sMax/sMax/sMax;
            
            % 15 minutes at scale = 10
            % X at scale = 1
            numA = 40000.0/scale;
            numB = 25000.0/scale;
            for a = -aMax:aMax/numA:aMax
                for b = -bMax:bMax/numB:bMax
                    ds = sMax/(clothSamples-1);
                    x=0.0; y = 0.0; t = 0.0; r = 0.0;
                    broke = false;
                    for i=1:clothSamples
                        s = ds*(i-1)*(a+b*ds*(i-1))*(ds*(i-1)-sMax);
                        t = t + s*ds;
                        dt = s*ds;
                        if i>1
                            x = x + cos(t-0.5*dt)*ds;
                            y = y + sin(t-0.5*dt)*ds;
                        end
                        r = r + s^2 * ds;
                        % Compute the curve. Break out of this loop, and then
                        % immediately continue to next iteration of the for b loop
                        % if tmax is exceeded in absolute value at any time.
                    end
                    if(broke == true); continue; end;
                    
                    q = atan2(y,x);
                    %if(~aTab.isInBounds(q,t)); continue; end;
                    % This is faster
                    if(q<qMin || q>=qMax || t<tMin || t>=tMax) ; continue; end;
                    if(n <= plotSamples)
                        plotArrayX(n) = x;
                        plotArrayY(n) = y;
                    elseif(n == plotSamples+1)
                        if(scale > 20)
                            figure(1);
                            plot(plotArrayX(1:n-1),plotArrayY(1:n-1),'.k','MarkerSize',3);
                            hold on;
                            fprintf('Plot array dumping\n');
                        end
                        n = 0;
                        elapsedTime = toc(startTic);
                        fprintf('Took %f minutes\n',elapsedTime/60.0);
                    end;
                    n = n + 1;
                    % Store coefficients.
                    r1Now = r1Tab.get(q,t);
                    r2Now = r2Tab.get(q,t);
                    if(abs(r1Now) > abs(r) && a >=0.0)
                        r1Tab.set(q,t,r);
                        a1Tab.set(q,t,a);
                        b1Tab.set(q,t,b);
                    elseif(abs(r2Now) > abs(r) && a <=0.0)
                        r2Tab.set(q,t,r);
                        a2Tab.set(q,t,a);
                        b2Tab.set(q,t,b);
                    end
                end
                
            end
            
            elapsedTime = toc(startTic);
            fprintf('Took %f minutes\n',elapsedTime/60.0);
            
            if(scale > 10)
                plot(plotArrayX(1:n-1),plotArrayY(1:n-1),'.k','MarkerSize',3);
            end
            
            save('cubicSpirals','a1Tab','a2Tab','b1Tab','b2Tab','r1Tab','r2Tab');
            
            figure(2);
            I1 = mat2gray(a1Tab.cellArray, [-aMax aMax]);
            imshow(I1);
            xlabel('Heading'); ylabel('Bearing'); title('a1Tab');
            figure(3);
            I2 = mat2gray(a2Tab.cellArray, [-aMax aMax]);
            imshow(I2);
            xlabel('Heading'); ylabel('Bearing'); title('a2Tab');
            figure(4);
            I1 = mat2gray(b1Tab.cellArray, [-bMax bMax]);
            imshow(I1);
            xlabel('Heading'); ylabel('Bearing'); title('b1Tab');
            figure(5);
            I2 = mat2gray(b2Tab.cellArray, [-bMax bMax]);
            imshow(I2);
            xlabel('Heading'); ylabel('Bearing'); title('b2Tab');
        end
    end
    
    
    methods(Access = public)
        function distance = getDistAtTime(obj, t)
            distance = interp1(obj.TArray, obj.distArray, t);
        end
        
        function linVel = getLinVelAtTime(obj, t)
            linVel = interp1(obj.TArray, obj.VArray, t);
        end
        
        function angVel = getAngVelAtTime(obj, t)
            angVel = interp1(obj.TArray, obj.wArray, t);
        end
        
        function pos = getPoseAtTime(obj, t)
            x = interp1(obj.TArray, obj.posArray(1,:), t);
            y = interp1(obj.TArray, obj.posArray(2,:), t);
            th = interp1(obj.TArray, obj.posArray(3,:), t);
            pos = pose(x, y, th);
        end
        
        function tf = getTrajectoryDuration(obj)
            tf = obj.TArray(end);
        end
    end
    
    
    methods(Static = true)
        
        function curve = planTrajectory(x,y,th,sgn)
            persistent inited;
            persistent a1T a2T b1T b2T r1T r2T;
            
            if(isempty(inited))
                load('cubicSpirals','a1Tab','a2Tab','b1Tab','b2Tab','r1Tab','r2Tab');
                inited = true;
                a1T = a1Tab;a2T = a2Tab;b1T = b1Tab;b2T = b2Tab;r1T = r1Tab;r2T = r2Tab;
            end
            
            q = atan2(y,x);
            % set pi(0 equal to - pi() to avoid out of bounds in scalar
            % field.
            if(abs(q-pi()) < 1e-10) ; q = -pi(); end;
            t = th;
            a1 = a1T.get(q,t);
            a2 = a2T.get(q,t);
            b1 = b1T.get(q,t);
            b2 = b2T.get(q,t);
            r1 = r1T.get(q,t);
            r2 = r2T.get(q,t);
            
            oneBad = false;
            twoBad = false;
            if(isinf(a1) || isinf(b1))
                oneBad = true;
            elseif(isinf(a2) || isinf(b2))
                twoBad = true;
            end;
            
            % Pick sole good one or the least curvy
            % I tried with and without this and it makes a huge difference!
            % Its the secret to computing a table in reasonable time because the
            % least curvy is pretty accurate
            if(oneBad==true && twoBad==true)
                disp('No solution\n');
                curve = {};
                return;
            elseif(oneBad==true)
                au=a2;bu=b2;
            elseif(twoBad==true)
                au=a1;bu=b1;
            elseif(r2 > r1)
                au=a1;bu=b1;
            else
                au=a2;bu=b2;
            end
            
            % Plot the corresponding unit
            su = 1.0;
            clothu = cubicSpiral([au bu su],201);
            
            %hold on;
            
            % Scale it up to the original
            xu = clothu.posArray(1,clothu.numSamples);
            yu = clothu.posArray(2,clothu.numSamples);
            Ru = sqrt(xu*xu+yu*yu);
            RO = sqrt(x*x+y*y);
            lam = RO/Ru;
            
            ss = lam;
            as = au/lam/lam/lam;
            bs = bu/lam/lam/lam/lam;
            
            if(sgn < 0)
                as = -as;
                ss = -ss;
            end
            curve = cubicSpiral([as bs ss],201);
        end
        
    end
end