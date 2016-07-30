function [rad2, po] = closestPointOnLineSegment(pi,p1,p2) % rad2 is a row with every point's squared dist
    % Given set of points and a line segment, returns the
    % closest point and square of distance to segment for
    % each point. If the closest point is an endpoint, returns
    % infinity for rad2 because such points are bad for
    % lidar matching localization.
    %
    % [rad2, po] = CLOSESTPOINTONLINESEGMENT(pi,p1,p2)
    %
    % pi - Array of points of size 2 x n.
    % p1 - Column of size 2, endpoint of segment.
    % p2 - Column of size 2, endpoint of segment.
    %
    % rad2 - Squared distance to closest point on segment.
    % po - Closest points on segment. Same size as pi.
    v1 = bsxfun(@minus,pi,p1); % vectors from p1 to pi
    v2 = p2-p1;                 %vectors from p1 to p2
    v3 = bsxfun(@minus,pi,p2);   % vectors from p2 to pi
    v1dotv2 = bsxfun(@times,v1,v2);  % v1 dot v2
    v1dotv2 = sum(v1dotv2,1);        % add y and x in every vector in v1dotv2
    v2dotv2 = sum(v2.*v2);           % dot and add x and y
    v3dotv2 = bsxfun(@times,v3,v2);
    v3dotv2 = sum(v3dotv2,1);
    nPoints = size(pi,2);          %number of points
    if ~nPoints
        display('pi is empty');
    end
    rad2 = zeros(1,nPoints);
    po = zeros(2,nPoints);
    % Closest is on segment
    flag1 = v1dotv2 > 0.0 & v3dotv2 < 0.0;   %condition to judge whether the point is on the line
    if any(flag1)                            % every point in the line range
        scale = v1dotv2/v2dotv2;
        temp = bsxfun(@plus,v2*scale,[p1(1) ; p1(2)]); % vector from p1 to the point corresponding to the pi on the line
        po(:,flag1) = temp(:,flag1);
        dx = pi(1,flag1)-po(1,flag1);
        %display(dx);
        dy = pi(2,flag1)-po(2,flag1);
        %display(dy);
        rad2(flag1) = dx.*dx+dy.*dy;
    end
    % Closest is first endpoint
    flag2 = v1dotv2 <= 0.0;
    if any(flag2)
        temp = bsxfun(@times,ones(2,sum(flag2)),[p1(1); p1(2)]);
        po(:,flag2) = temp;
        rad2(flag2) = inf;
    end
    % Closest is second endpoint
    flag3 = ones(size(1,nPoints)) & ~flag1 & ~flag2;
    if any(flag3)
        temp = bsxfun(@times,ones(2,sum(flag3)),[p2(1); p2(2)]);
        po(:,flag3) = temp;
        rad2(flag3) = inf;
    end
end