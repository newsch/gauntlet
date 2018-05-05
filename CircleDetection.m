function [center, inliers, outliers] = CircleDetection(x,y,debug)
    % testgetDist()
    %load playpensample.mat
%     [ctheta, cr] = cleanData(theta,r);
%     [x,y] = polar2cart(deg2rad(ctheta),cr);
    figure; hold on
    plot(x,y,'ks')
    plot(0,0,'kO')
    legendata = ["lidar data","neato location"];
    data = [x y];
    radius = 0.360892/2;
    [pts, center, inliers,outliers] = Circlefit(x, y, 0.01, 5000, radius);
    if debug
        plot(inliers(:,1), inliers(:,2), 'r*')
        plot(pts(:, 1), pts(:, 2), 'g*')
        viscircles(center, radius)
        axis('equal')
        legend('lidar points', 'neato', 'inliers', 'refernece points')
    end

    
    function [pts, center,inliers,outliers] = Circlefit(x, y, d, n, r)
    %ROBUSTLINEFIT  Fit lines to scanner data with RANSAC
        data = [x y];
        choices = randsample(length(data),3*n,'true');
        for i = 3:3:length(choices)
            p1 = data(choices(i-2),:);
            p2 = data(choices(i - 1),:);
            p3 = data(choices(i), :);
            center = calc_circle(p1, p2, p3);
            D = getDist(center, data);  % get distances of all data points to line
            results.inliers(i / 3) = sum(abs(D - r) < d);
            results.distances(:,i / 3) = D;
        end

        [sR, sI] = sort(results.inliers,'descend');
        sP2 = data(choices(sI(1)*3),:);
        sP1 = data(choices(sI(1)*3 - 1),:);
        sP3 = data(choices(sI(1)*3 - 2),:);

    %     figure; hold on
    %     plot(data(:,1),data(:,2),'ks')  % cleaned lidar data
    %     for i = 1:3
    %         quiver(sP1(i,1),sP1(i,2),sP2(i,1)-sP1(i,1),sP2(i,2)-sP1(i,2),'LineWidth',2)
    %     end
    %     legend('data','line 1','line 2','line 3')
        pts = [sP1; sP2; sP3];
        center = calc_circle(sP2, sP1, sP3);
        D = results.distances(:,sI(1));
        inliers = data(abs(D - r) < d, :);
        outliers = data(abs(D - r) > d, :);

    end

    function center = calc_circle(p1, p2, p3)
        mid1 = (p2+p1) / 2;
        mid2 = (p3+p2) / 2;
        slope1 = -(p2(1) - p1(1)) / (p2(2)-p1(2));
        slope2 = -(p3(1) - p2(1)) / (p3(2)-p2(2));
        system = rref([1, -slope1, mid1(2) - (slope1 * mid1(1));
                  1, -slope2, mid2(2) - (slope2 * mid2(1))]);
        center = [system(2, 3), system(1,3)];
    end

    function dist = getDist(p1,p2)
    % GETDIST  finds the perpendicular distance from p3 to the line
    % through p1 and p2.
    % p3 is a collection of x and y points of format [X Y]

        p1 = ones([size(p2, 1), 1]) * p1;
        dist_vec = p2 - p1;
        dist = sqrt(sum(dist_vec.^2, 2));

    end

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

end