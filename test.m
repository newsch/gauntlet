sub = rossubscriber('/stable_scan');

lidar_to_wheels = 3.4/12;

rotation = @(theta) [cos(theta), sin(theta), 0;
            -sin(theta), cos(theta), 0;
            0, 0, 1];
translation = @(X, Y) [1, 0, -X;
               0, 1, -Y;
               0, 0, 1];

scan_message = receive(sub);
r = scan_message.Ranges(1:end-1);
theta = [0:359]';
[ctheta, cr] = cleanData(theta,r);
[x,y] = polar2cart(deg2rad(ctheta),cr);

data = [x*3.28084,y*3.28084,ones([length(x),1])];
%data = translation(neato_pos) * rotation(neato_ori) * translation(lidar_to_wheels) * data;
x = data(:,1);
y = data(:,2);

% [center, inliers, outliers] = CircleDetection(x,y,true);
% x = outliers(:,1);
% y = outliers(:,2);
endpoints = segment_ransac(x,y,true);
sorted_ends = sort(endpoints(:,5));
disp(sorted_ends)
disp(sum(sorted_ends))
disp(size(sorted_ends));
disp(sum(sorted_ends((sorted_ends < 15))));




function [ctheta,cr] = cleanData(theta, r)
    nonzero_r = r ~= 0;
    close_r = r < 5;
    i_clean = nonzero_r & close_r;  % indices of clean data
    ctheta = theta(i_clean);
    cr = r(i_clean);
end

function [X,Y] = polar2cart(theta,r)
    X = r.*cos(theta);
    Y = r.*sin(theta);
end