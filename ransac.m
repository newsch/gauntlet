% testgetDist()
load scan4.mat
robustLineFit(theta,r,1,20)

function testgetDist()
    p1 = [0 0];
    p2 = [1 1];
    p3 = [0 1];
    getDist(p1,p2,p3)  % should be sqrt(2)/2
    
    p1 = [3 3];
    p2 = [3 -3];
    p3 = [2 0];
    getDist(p1,p2,p3)  % should be 1
    
    p1 = [0 0];
    p2 = [1 1];
    p3 = [0 1; 1 0; 2 2];
    getDist(p1,p2,p3)
end

function robustLineFit(theta, r, d, n)
%ROBUSTLINEFIT  Fit lines to scanner data with RANSAC
    [ctheta, cr] = cleanData(theta,r);
    [x,y] = polar2cart(deg2rad(ctheta),cr);
    data = [x y];
    choices = randsample(length(data),2*n);
    tic;
    for i = 2:2:length(choices)
        p1 = data(choices(i-1),:)
        p2 = data(choices(i),:)
        D = getDist(p1,p2,data);  % get distances of all data points to line
        results(i / 2) = sum(D > d);
    end
    toc
    [sR, sI] = sort(results,'descend');
    sP2 = data(choices(sI*2),:);
    sP1 = data(choices(sI*2 - 1),:);
    
    figure; hold on
    plot(data(:,1),data(:,2),'ks')  % cleaned lidar data
    for i = 1:3
        quiver(sP1(i,1),sP1(i,2),sP2(i,1)-sP1(i,1),sP2(i,2)-sP1(i,2),'LineWidth',2)
    end
    legend('data','1','2','3')
end

function dist = getDist(p1,p2,p3)
% GETDIST  finds the perpendicular distance from p3 to the line
% through p1 and p2.
% p3 is a collection of x and y points of format [X Y]
    
    line_points = num2cell([p1 p2]);
    [x1,y1,x2,y2] = deal(line_points{:});
    x0 = p3(:,1);
    y0 = p3(:,2);
    dist = abs((y2-y1).*x0 - (x2-x1).*y0 + x2.*y1 - y2.*y1) / sqrt((y2-y1).^2 + (x2-x1).^2);
end

function [ctheta,cr] = cleanData(theta, r)
    i_clean = r ~= 0; % indices of clean data
    ctheta = theta(i_clean);
    cr = r(i_clean);
end

function [X,Y] = polar2cart(theta,r)
    X = r.*cos(theta);
    Y = r.*sin(theta);
end

