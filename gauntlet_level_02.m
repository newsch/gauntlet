function run = runCourse(DRYRUN)
% GAUNTLET_LEVEL_02.m  Run the gauntlet with no obstacles and known BoB
    %% gauntlet info
    init_pos = [0,0];
    init_head = 0;

    bob_pos = [7, 4.5];
    box_pos = [2 3; 4 4.5; 5.5 2; 0 4.5];
    walls_pos = [-1,-1, -1,5.5; -1,5.5, 8,5.5; 8,5.5, 8,-1; 8,-1, -1,-1];  % rows of two points defining wall line segments
    
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
    r = 0.1;  % resolution
    x = -3:r:10;
    y = x;
    [X,Y] = meshgrid(x,y);
    % build course
    %Z = point2field(bob_pos,X,Y,exp(1),1);
    Z = zeros(size(X));
    for i = 1:length(box_pos)
        %Z = Z - point2field(box_pos(i,:),X,Y,exp(1),1);
    end
    % add walls
    wall_points = []
    for i = 1:length(walls_pos)
        p1 = walls_pos(i,1:2);
        p2 = walls_pos(i,3:4);
        [newZ, points] = line2field(p1,p2,X,Y,r);
        disp(size(wall_points))
        Z = Z - newZ;
        wall_points = [wall_points; points];
        
        
    end
    figure;
    plot(wall_points(:,1), wall_points(:,2), 'rs');
    figure;
    surf(X,Y,Z)
    shading interp
%     ylim([-1 5.5])
%     xlim([-1 8])
    %figure;
    %contour(X,Y,Z,20)
%     ylim([-1 5.5])
%     xlim([-1 8])
    
    [Gx,Gy] = gradient(Z);
    
    %% calculate and run path
    rot = atan2(bob_pos(2),bob_pos(1));
    dist = getDistance(init_pos,bob_pos);

    % rotate
    if rot > 0
        Vr = rs;
        Vl = -rs;
    else
        Vr = -rs;
        Vl = rs;
    end
    T = (rot / ((Vr - Vl)/d));
    drive(Vl,Vr,T)

    % advance
    vs = 0.15*3.2;
    Vr = vs;
    Vl = vs;
    T = (dist / vs);
    drive(Vl,Vr,T)
%     setVel(0,0)

    %% functions
    function Z = point2field(p1,X,Y, scale,style)
        if style == 1
            Z = log(sqrt((X - p1(1)).^2 + (Y - p1(2)).^2))/log(scale);
        else
           Z = 0.1 * (sqrt((X - p1(1)).^2 + (Y - p1(2)).^2)).^3;
        end
    end

    function [Z,points] = line2field(p1,p2,X,Y,r)
        Z = 0;
        dy = p2(2) - p1(2);
        dx = p2(1) - p1(1);
        num_points = round(getDistance(p1,p2) / r);
        points = [];
        for n = 0:num_points
            points = [points; p1 + [dx,dy]/num_points*n];
            
            Z = Z + point2field(p1 + [dx,dy]/num_points*n, X, Y, exp(1), 1);
        end
    end

    function drive(vl,vr,t)
        run.wheel_vel = [run.wheel_vel; [vl,vr]];
        run.times = [run.times; t];
        if ~DRYRUN
            setVel(vl,vr)
            pause(t)
        end
    end

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