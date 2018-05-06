function run = runCourse()
% GAUNTLET_LEVEL_04.m  Run the gauntlet with no obstacles and known BoB
% radius
    %% gauntlet info
    PLOT = true;
%     init_pos = [2,0];
    init_head = 0;

    bob_pos = [7, 4.5];
    
%     run.init.pos = init_pos;
    run.init.head = init_head;
    run.bob.pos = bob_pos;
    
    run.wheel_vel = [];
    run.times = [];
    
    fpm = 3.28084;
    
    %% setup
    max_speed = 0.3;
    d = 0.24*fpm;
    rs = 0.15*fpm;
    vs = 0.15*fpm;
    
    % field parameters
    r = 0.05;  % resolution
%     x = -20:r:20;
%     y = -20:r:20;
    x = -10:r:10;
    y = -10:r:10;
%     x = -1:r:8;
%     y = -1:6;
    [X,Y] = meshgrid(x,y);
    
    % path/location parameters
    a = 0.5;  % step size
    pos(:,1) = [0;0];  % initial position
    head(1) = 0;  % initial heading
    
    rs = 0.1*fpm;  % rotation speed
    vs = 0.15*fpm;  % linear velocity speed

    
    % lidar parameters, functions
    sub = rossubscriber('/stable_scan');

    lidar_to_wheels = 3.4/12;

    rotation = @(theta) [cos(theta), sin(theta), 0;
                -sin(theta), cos(theta), 0;
                0, 0, 1];
    translation = @(X, Y) [1, 0, -X;
                   0, 1, -Y;
                   0, 0, 1];
    %% run
    for j = 1:8
        ci = length(head);  % current index for heading/position
        neato_pos = pos(:,ci);
        neato_ori = head(ci);
        % current position = pos(:,end)
        % current heading = head(end)
        %% scan lidar
        scan_message = receive(sub);
        lidr = scan_message.Ranges(1:end-1);
        lidtheta = (0:359)';
        [ctheta, cr] = cleanData(lidtheta,lidr);
        [lidx,lidy] = polar2cart(deg2rad(ctheta),cr);

        data = [lidx*fpm,lidy*fpm,ones([length(lidx),1])];  % convert to feet
        adata = (translation(-lidar_to_wheels, 0) * data')';
        bdata = (rotation(-neato_ori) * adata')';
        cdata = (translation(-neato_pos(1),-neato_pos(2)) * bdata')';
        
        %cdata = (translation(neato_pos(1),neato_pos(2)) * rotation(-neato_ori) * translation(lidar_to_wheels, 0) * data')';
        glox = cdata(:,1);
        gloy = cdata(:,2);

        %[center, inliers, outliers] = CircleDetection(x,y,true)
        %x = outliers(:,1);
        %y = outliers(:,2);
        endpoints = segment_ransac(glox,gloy,true);
        %% generate field
        walls_pos = endpoints(:, 1:4);
        %walls_pos = endpoints(endpoints(:,end) > 14, 1:4); %get segments with more than 14 points in them
        Z = point2field(bob_pos,X,Y,exp(1))*14;  % begin w/ BoB position
        % add walls
        all_peaks = [];
        for i = 1:length(walls_pos(:,1))
            p1 = walls_pos(i,1:2);
            p2 = walls_pos(i,3:4);
            [peaks, new_z] = line2field(p1,p2,X,Y,0.1); 
            Z = Z - new_z;
            all_peaks = [all_peaks; peaks];
        end
        [Gx,Gy] = gradient(Z,r);
        
        if PLOT
        % plot new contour with lidar data
        figure; hold on
        contour(X,Y,Z,100,'DisplayName','contour')
        %plot(walls_pos(:, 1), walls_pos(:,2), 'mo', 'MarkerSize', 4);
        %plot(walls_pos(:, 3), walls_pos(:,4), 'mo', 'MarkerSize', 4);
        %plot(center(1), center(2), 'kx', 'MarkerSize', 5);
        plot(all_peaks(:,1), all_peaks(:,2), 'mo');
        plot(lidx*fpm,lidy*fpm,'sk','DisplayName','uncorrected lidar')
%         plot(adata(:,1), adata(:,2), 'b*','DisplayName','wheelbase trans');
%         plot(bdata(:,1), bdata(:,2), 'g*','DisplayName','rotated');
        plot(cdata(:,1), cdata(:,2), 'r*','DisplayName','position trans');
        %plot(glox,gloy,'*k')
        plot(neato_pos(1),neato_pos(2),'ok','DisplayName','neato pos')
%         axis equal
%         legend('contour','uncorrected lidar','wheelbase trans', 'rotated', 'position trans','neato pos')
        end
        %% calculate path
        for i = ci+1:ci + 4
            cpos = pos(:,i-1);
            xi = find_nearest(cpos(1), X(1,:));  % index of x pos
            yi = find_nearest(cpos(2), Y(:,1));  % index of y pos
            g = -[Gx(yi,xi);Gy(yi,xi)];  % flip gradient
            disp(g)
            head(i) = atan2(g(2),g(1));  % preserve direction
            pos(:,i) = cpos + (g/norm(g)).*a;
        end
        %% plot path
        if PLOT
%         figure; hold on;
%         contour(X,Y,Z,100)
        plot(pos(1,:),pos(2,:),'*-','DisplayName','path points')
        plot(pos(1,1),pos(2,1),'*g','DisplayName','initial position')
        plot(pos(1,ci),pos(2,ci),'*y','DisplayName','current position')
        quiver(pos(1,:),pos(2,:),cos(head),sin(head),'DisplayName','path headings')
        title("Desired path")
        xlabel("X position")
        ylabel("Y position")
%         ylim([-1 5.5])
%         xlim([-1 8])
        hold off;
        axis equal
        legend()
        end
        %% Calculate commands
        %dist = vecnorm(diff(pos(:,ci:end),1,2));
        dist = sqrt(sum(diff(pos(:,ci:end),1,2).^2));
        rot = diff(head(ci:end));
        Vr = [];
        Vl = [];
        T = [];
        for i = 1:length(rot)
            % rotate
            if rot(i) > 0
                Vr = [Vr rs];
                Vl = [Vl -rs];
            else
                Vr = [Vr -rs];
                Vl = [Vl rs];
            end
            T = [T, (rot(i) / ((Vr(end) - Vl(end))/d))];
            % advance
            Vr = [Vr vs];
            Vl = [Vl vs];
            T = [T, (dist(i) / vs)];
        end
        % convert to meters for neato
        Vl = Vl ./ fpm;
        Vr = Vr ./ fpm;
        %% run course
        fprintf("Max Vr: \t%.3f\n",max(Vr))
        fprintf("Max Vl: \t%.3f\n",max(Vl))
        fprintf("Min time: \t%.6f\n",min(T))
        if min(T) < 0
            disp("WARNING: Times less than zero")
        end
        if max(Vr) > max_speed || max(Vl) > max_speed
            disp("WARNING: Velocities greater than max of "+string(max_speed)+".")
        end

        run.wheel_vel = [Vl',Vr'];
        run.times = T';
        % Run course if not a dry run
%         if ~DRYRUN
            drive(T,Vl,Vr)
%         end
    end
    
    %% functions
    function move(vl,vr,t)
        run.wheel_vel = [run.wheel_vel; [vl,vr]];
        run.times = [run.times; t];
        if ~DRYRUN
            setVel(vl,vr)
            pause(t)
        end
    end

    function stop(pub)
    % STOP  stop the robot
        disp("Stopping robot")
        message = rosmessage(pub);
        message.Data = [0 0];
        send(pub, message);
    end

    function setVel(vl, vr)
    % SETVEL  set the wheel velocities (in feet)
        message = rosmessage(vel_pub);
        message.Data = [vl vr] / fpm;
        send(vel_pub, message);
    end

end

function [ctheta,cr] = cleanData(theta, r)
    nonzero_r = r ~= 0;
%     close_r = r < 5;
    i_clean = nonzero_r; % & close_r;  % indices of clean data
    ctheta = theta(i_clean);
    cr = r(i_clean);
end

function [X,Y] = polar2cart(theta,r)
    X = r.*cos(theta);
    Y = r.*sin(theta);
end

function d = getDistance(p1,p2)
% GETDISTANCE  get the distance between two points
    points = num2cell([p1,p2]);
    [x1,y1,x2,y2] = deal(points{:});
    d = sqrt((x2-x1)^2 + (y2-y1)^2);
end

function i = find_nearest(val,range)
    [v,i] = min(abs(range - val));
end

function Z = point2field(p1,X,Y, scale)
        Z = log(sqrt((X - p1(1)).^2 + (Y - p1(2)).^2))/log(scale);
end

function [peak_points, Z] = line2field(p1,p2,X,Y,r)
    Z = zeros(size(X));
    dy = p2(2) - p1(2);
    dx = p2(1) - p1(1);
    num_points = round(getDistance(p1,p2) / r) + 1;
    peak_points = [];
    for n = 1:num_points
        peak_points = [peak_points; p1 + [dx,dy]/num_points*n];
        Z = Z + point2field(p1 + [dx,dy]/num_points*n, X, Y, exp(1));%/num_points;
    end
end

function drive(Times,Vl,Vr)
%RUNCOURSE  Run the robot through a precalculated course
%   Times is an array of pause lengths between commands.
%   Vl is an array of speeds for the left wheel.
%   Vr is an array of speeds for the right wheel.
    
    cleanupObj = onCleanup(@cleanMeUp);

    sub = rossubscriber('/bump');
    pub = rospublisher('/raw_vel');
    message = rosmessage(pub);

    disp("Ready to run course.")
    disp("Press any key to start.")
    disp("Press CTRL+C to stop.")
    pause();  % wait for input
    disp("Starting run.")
    
    % loop through times and velocities
    for i = 1:length(Times)
        message.Data = [Vl(i), Vr(i)];
        send(pub, message);
        pause(Times(i));
    end

    function cleanMeUp()
    % Called when the function is aborted or finished.
        disp("Stopping robut.")
        message.Data = [0,0];
        send(pub,message);
    end
end