function run = runCourse(DRYRUN)
% GAUNTLET_LEVEL_01.m  Run the gauntlet with no obstacles and known BoB
    %% gauntlet info
    init_pos = [0,0];
    init_head = 0;

    bob_pos = [7, 4.5];
    
    run.init.pos = init_pos;
    run.init.head = init_head;
    run.bob.pos = bob_pos;
    
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
    function drive(vl,vr,t)
        run.wheel_vel = [run.wheel_vel; [vl,vr]];
        run.times = [run.times; t];
        if ~DRYRUN
            setVel(vl,vr)
            pause(t)
        end
    end

    function d = getDistance(p1,p2);
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