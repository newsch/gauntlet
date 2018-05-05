sub = rossubscriber('/stable_scan');
scan_message = receive(sub);
r = scan_message.Ranges(1:end-1);
theta = [0:359]';
endpoints = segment_ransac(theta, r);