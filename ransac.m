testRobustLineFit()

function testRobustLineFit()
    p1 = [0 0];
    p2 = [1 1];
    p3 = [0 1];
    getDist(p1,p2,p3)  % should be sqrt(2)/2
    
    p1 = [3 3];
    p2 = [3 -3];
    p3 = [2 0];
    getDist(p1,p2,p3)  % should be 1
    
end

function robustLineFit(theta, r, d, n)
%ROBUSTLINEFIT  Fit lines to scanner data with RANSAC
    [ctheta, cr] = cleanData(theta,r);
    data = polar2cart(ctheta,cr);
    choices = randsample(length(data),2*n);
    for i = 1:2:length(choices)
        p1 = data(choices(i),:)
        p2 = data(choices(i+1),:)
    end
   
end

function dist = getDist(p1,p2,p3)
% GETDIST  finds the perpendicular distance from p3 to the line
% through p1 and p2
    points = num2cell([p1 p2 p3]);
    [x1,y1,x2,y2,x0,y0] = deal(points{:});
    dist = abs((y2-y1)*x0 - (x2-x1)*y0 + x2*y1 - y2*y1) / sqrt((y2-y1)^2 + (x2-x1)^2);
            
end

function [ctheta,cr] = cleanData(theta, r)
    i_clean = r > 0; % indices of clean data
    ctheta = theta(i_clean);
    cr = r(i_clean);
end

function [X,Y] = polar2cart(theta,r)
    X = r.*cos(theta);
    Y = r.*sin(theta);
end

