%function [a1Tab,a2Tab ,b1Tab ,b2Tab ,r1Tab ,r2Tab,xTab] =  makeLookupTable(scale)
% Make 6 lookup tables to be used for cubicSpiral generation.
% Principle is to sample densely in coefficient space and
% integrate the curvature to find the curve. Then store the
% result in a table that inverts the mapping.
% On a 2.6 GHz core i7:
% Scale = 100 takes 0.17 minutes
% Scale = 50 takes 0.64 minutes
% Scale = 10 takes 15 minutes
% Scale = 1 takes > 12 hours
scale = 50;
startTic = tic();
sMax = 1.0; % length of curve
qMax = pi(); % max bearing
qMin = -pi(); % min bearing
tMax = 1.5*pi(); % max heading
tMin = -1.5*pi(); % min heading
numT = round(10*126/scale); % heading samples
numQ = round(50*126/scale); % bearing samples

% Do a quick performance estimate
radius = 1.0;                % what is this used for?
ds = radius*(qMax-qMin)/numQ;
dt = (tMax-tMin)/numT;
fprintf('Error predicted %f m. in translation, %f rads in rotation\n',ds,dt);

% Allocate tables
a1Tab = scalarField(qMin,qMax,numQ,tMin,tMax,numT);
b1Tab = scalarField(qMin,qMax,numQ,tMin,tMax,numT);
r1Tab = scalarField(qMin,qMax,numQ,tMin,tMax,numT); % energy
a2Tab = scalarField(qMin,qMax,numQ,tMin,tMax,numT);
b2Tab = scalarField(qMin,qMax,numQ,tMin,tMax,numT);
r2Tab = scalarField(qMin,qMax,numQ,tMin,tMax,numT); % energy
xTab = scalarField(qMin,qMax,numQ,tMin,tMax,numT);

clothSamples = 201;

plotSamples = 10000;             % what is it used for?
plotArrayX = zeros(plotSamples);
plotArrayY = zeros(plotSamples);

n = 1;                        % what is it used for?
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
            x = x + cos(t)*ds;
            y = y + sin(t)*ds;
            r = r + s^2 * ds;
            % Compute the curve. Break out of this loop, and then
            % immediately continue to next iteration of the for b loop
            % if tmax is exceeded in absolute value at any time.
            % FILL THIS IN end
            q = atan2(y,x);
            if(q<qMin || q>=qMax || t<tMin || t>=tMax)
                broke = true;
                continue;
            end     % which is faster?
        end% This is faster
        %~a1Tab.isInBounds(q,t) || ~b1Tab.isInBounds(q,t)
        %if(q<qMin || q>=qMax || t<tMin || t>=tMax) ; continue; end;
        % Store coefficients. First read what we have now.
        if broke == false
            xTab.set(q,t,x);
            if a >=0
                r1Now = r1Tab.get(q,t);
                if r1Now > r
                    a1Tab.set(q,t,a);
                    b1Tab.set(q,t,b);
                    r1Tab.set(q,t,r);
                end
            end
            if a<=0                     % why the situation of a ==0 needs to exist on both matrix
                r2Now = r2Tab.get(q,t);
                if r2Now > r
                    a2Tab.set(q,t,a);
                    b2Tab.set(q,t,b);
                    r2Tab.set(q,t,r);
                end
            end
        end
        % etc for other tables
        % add more stuff here……
        
    end
    % more code ommitted
end



