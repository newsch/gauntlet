function plot_path(Vl,Vr,T)
    d = 0.24;

%     figure;
%     hold on
%     pos = zeros(length(Vl), 3); %x, y, theta
%     dt = T;
%     lin_vel = (Vl+ Vr)/2;
%     ang_vel = (Vr - Vl)/d;
% 
%     for i = 2:1:length(dt)
%         T_hat = [cos(pos(i - 1, 3)), sin(pos(i - 1, 3))];
%         pos(i, 1) = (lin_vel(i) * dt(i) * T_hat(1)) + pos(i - 1, 1);
%         pos(i, 2) = (lin_vel(i) * dt(i) * T_hat(2)) + pos(i - 1, 2);
%         pos(i, 3) = (ang_vel(i) * dt(i)) + pos(i - 1, 3);
%     end
%     plot(pos(:, 1), pos(:, 2), 'ro')
%     title('hi')

    figure; hold on
%     plot(r(:,1),r(:,2))
    pos = [0 0];  % position of robot
    head = 0;  % heading of robot
    plot(pos(1),pos(2),'b*')
    for i = 1:length(T)
        dt = T(i);
        That = [cos(head), sin(head)];
        lin_vel = (Vl(i) + Vr(i))/2;
        ang_vel = (Vr(i) - Vl(i))/d;
        drdpos = lin_vel*That;
        drdhead = ang_vel;
        plot(pos(1),pos(2),'r*')
        quiver(pos(1),pos(2), That(1),That(2))
        new_pos = pos + drdpos*dt;
        new_head = head + drdhead*dt;
        pos = new_pos;
        head = new_head;
    end
    plot(pos(1),pos(2),'g*')
    axis equal
end