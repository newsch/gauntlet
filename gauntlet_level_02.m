function run = runCourse(DRYRUN)
% GAUNTLET_LEVEL_02.m  Run the gauntlet with no obstacles and known BoB
    %% gauntlet info
    init_pos = [0,0];
    init_head = 0;

    bob_pos = [7, 4.5];
    box_pos = [2 3; 4 4.5; 5.5 2];
    walls_pos = [-1,-1, -1,5.5; -1,5.5, 8,5.5; 8,5.5, 8,-1; 8,-1, -1,-1];  % rows of two points defining wall line segments
%     walls_pos = [8,5.5,8,-1; 8,-1, -1,-1];
    
    run.init.pos = init_pos;
    run.init.head = init_head;
    run.bob.pos = bob_pos;
    run.boxes.pos = box_pos;
    run.walls.pos = walls_pos;
    
    run.wheel_vel = [];
    run.times = [];

    %% setup
    max_speed = 0.3;
    d = 0.24*3.2;
    
    rs = 0.15*3.2;
    vs = 0.15*3.2;
    
    if ~DRYRUN
        disp("Connecting to robut.")
        vel_pub = rospublisher('/raw_vel');

        cleanUp = @() stop(vel_pub);  % stop on exit/error
        cleanupObj = onCleanup(cleanUp);
    end
    
    % calculate course
    r = 0.05;  % resolution
    x = -10:r:10;
    y = -10:r:10;
    [X,Y] = meshgrid(x,y);
    % build course
    Z = point2field(bob_pos,X,Y,exp(1))*8;
    for i = 1:length(box_pos)
        Z = Z - point2field(box_pos(i,:),X,Y,exp(1))*1;
    end
    % add walls
    for i = 1:length(walls_pos(:,1))
        p1 = walls_pos(i,1:2);
        p2 = walls_pos(i,3:4);
        Z = Z - line2field(p1,p2,X,Y,0.5)*2;
    end
    % remove Inf
%     Z(Z==Inf) = 100;
%     Z(Z==-Inf) = -100;

    figure;
    surf(X,Y,Z)
    shading interp
    ylim([-1 5.5])
    xlim([-1 8])
    figure; hold on
    contour(X,Y,Z,100)
    
    [Gx,Gy] = gradient(Z,r);
    quiver(X,Y,-Gx,-Gy,10)
%     hold off
    
    %% Calculating course
    a = 0.25;  % step size
    pos(:,1) = [0;0];
    head(1) = 0;
    for i = 2:40
        cpos = pos(:,i-1);
        xi = find_nearest(cpos(1), X(1,:));  % index of x pos
        yi = find_nearest(cpos(2), Y(:,1));  % index of y pos
        g = -[Gx(yi,xi);Gy(yi,xi)];  % flip gradient
        head(i) = atan2(g(2),g(1));  % preserve direction
        pos(:,i) = cpos + (g/vecnorm(g)).*a;
    end
    
    %% Plot desired path
%     figure; hold on;
%     contour(X,Y,Z,100)
    plot(pos(1,:),pos(2,:),'*-')
    plot(pos(1,1),pos(2,1),'*g')
    quiver(pos(1,:),pos(2,:),cos(head),sin(head))
    title("Desired path")
    xlabel("X position")
    ylabel("Y position")
    ylim([-1 5.5])
    xlim([-1 8])
%     axis equal

    %% Calculate commands
    dist = vecnorm(diff(pos,1,2));
    rot = diff(head);
    Vr = [];
    Vl = [];
    T = [];
    for i = 1:length(rot)
        % rotate
        rs = 0.15*3.2;  % rotation speed
        if rot(i) > 0
            Vr = [Vr rs];
            Vl = [Vl -rs];
        else
            Vr = [Vr -rs];
            Vl = [Vl rs];
        end
        T = [T (rot(i) / ((Vr(end) - Vl(end))/d))];
        % advance
        vs = 0.15*3.2;  % linear velocity speed
        Vr = [Vr vs];
        Vl = [Vl vs];
        T = [T (norm(dist(:,i)) / vs)];
    end

    Vl = Vl ./ 3.2;
    Vr = Vr ./ 3.2;
    
    %% Plot calculated course
    figure; hold on
    title("Predicted path")
    xlabel("X position")
    ylabel("Y position")
    axis equal

    contour(X,Y,Z)

    cpos = pos(:,1);
    chead = head(1);
    for i = 1:length(T)
        dt = T(i);
        That = [cos(chead); sin(chead)];
        lin_vel = (Vl(i)*3.2 + Vr(i)*3.2)/2;
        ang_vel = (Vr(i)*3.2 - Vl(i)*3.2)/(d);
        drdcpos = lin_vel*That;
        drdchead = ang_vel;
        plot(cpos(1),cpos(2),'r*')
        quiver(cpos(1),cpos(2), That(1),That(2),'b')
        new_cpos = cpos + drdcpos*dt;
        new_chead = chead + drdchead*dt;
        cpos = new_cpos;
        chead = new_chead;
    end

    %% Run course    
    fprintf("Max Vr: \t%.3f\n",max(Vr))
    fprintf("Max Vl: \t%.3f\n",max(Vl))
    if min(T) < 0
        disp("WARNING: Times less than zero")
    end
    if max(Vr) > max_speed || max(Vl) > max_speed
        disp("WARNING: Velocities greater than max of "+string(max_speed)+".")
    end
    
    run.wheel_vel = [Vl',Vr'];
    run.times = T';
    % Run course if not a dry run
    if ~DRYRUN
        drive(T,Vl,Vr)
    end
    
    %% functions
    function i = find_nearest(val,range)
        [v,i] = min(abs(range - val));
    end
    
    function Z = point2field(p1,X,Y, scale)
            Z = log(sqrt((X - p1(1)).^2 + (Y - p1(2)).^2))/log(scale);
    end

    function [Z] = line2field(p1,p2,X,Y,r)
        Z = 0;
        dy = p2(2) - p1(2);
        dx = p2(1) - p1(1);
        num_points = round(getDistance(p1,p2) / r);
        for n = 0:num_points
           Z = Z + point2field(p1 + [dx,dy]/num_points*n, X, Y, exp(1));%/num_points;
        end
    end

%     function drive(vl,vr,t)
%         run.wheel_vel = [run.wheel_vel; [vl,vr]];
%         run.times = [run.times; t];
%         if ~DRYRUN
%             setVel(vl,vr)
%             pause(t)
%         end
%     end

    function d = getDistance(p1,p2)
    % GETDISTANCE  get the distance between two points
        points = num2cell([p1,p2]);
        [x1,y1,x2,y2] = deal(points{:});
        d = sqrt((x2-x1)^2 + (y2-y1)^2);
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
        message.Data = [vl vr] / 3.2;
        send(vel_pub, message);
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