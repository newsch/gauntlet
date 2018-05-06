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

xpos = 1;
ypos = 1;

data = [x,y,ones([length(x),1])]*3.2;  % convert to feet
data = (translate(xpos, ypos) * data')';