function [center, inliers, outliers] = CircleDetection_c4(x,y,debug)
    % testgetDist()
    %load playpensample.mat
%     [ctheta, cr] = cleanData(theta,r);
%     [x,y] = polar2cart(deg2rad(ctheta),cr);
    min_arc_pts = 8;
    data = [x y];
    radius = 11/24;
    [pts, center, radius, inliers,outliers] = Circlefit(x, y, 0.003);
    figure; hold on
    for i = 1:size(data, 1)
       plot(data(i, 1), data(i, 2),'bo');
       %pause(0.1)
    end
    if debug
        figure; hold on
        plot(x,y,'ks')
        plot(0,0,'kO')
        axis('equal')
        legendata = ["lidar data","neato location"];
        plot(inliers(:,1), inliers(:,2), 'r*')
        plot(pts(:, 1), pts(:, 2), 'g*')
        viscircles(center, radius)
        axis('equal')
        legend('lidar points', 'neato', 'inliers', 'refernece points')
    end

    
    function [pts, center, r, inliers,outliers] = Circlefit(x, y, d)
    %ROBUSTLINEFIT  Fit lines to scanner data with RANSAC
        data = [x y];
        
        for i = 1:size(data,1)
            p1 = [data(mod(i, size(data, 1))+1, 1), data(mod(i, size(data, 1))+1,2)];
            p2 = [data(mod(i+2,size(data,1))+1, 1), data(mod(i+2,size(data,1))+1,2)];
            p3 = [data(mod(i+4,size(data,1))+1, 1), data(mod(i+4,size(data,1))+1,2)];
            [center,r] = calc_circle(p1,p2,p3);
            D = getDist(center, data);  % get distances of all data points to line
            if r < 0.5
                results.inliers(i) = sum(abs(D - r) < d);
            else
               results.inliers(i) = 0;
            end
            results.distances(:,i) = D;
        end

        [sR, sI] = sort(results.inliers,'descend');
        sP2 = data((sI(1)),:);
        sP1 = data((sI(1)+2),:);
        sP3 = data((sI(1)+4),:);

        pts = [sP1; sP2; sP3];
        [center,r] = calc_circle(sP2, sP1, sP3);
        D = results.distances(:,sI(1));
        inliers = data(abs(D - r) < d, :);
        outliers = data(abs(D - r) > d, :);
        if ((length(inliers) < min_arc_pts))
           inliers = [];
           outliers = data;
           center = [];
           r= [];
        end
        

    end

    function [center,r] = calc_circle(p1, p2, p3)
        mid1 = (p2+p1) / 2;
        mid2 = (p3+p2) / 2;
        slope1 = -(p2(1) - p1(1)) / (p2(2)-p1(2));
        slope2 = -(p3(1) - p2(1)) / (p3(2)-p2(2));
        system = rref([1, -slope1, mid1(2) - (slope1 * mid1(1));
                  1, -slope2, mid2(2) - (slope2 * mid2(1))]);
        center = [system(2, 3), system(1,3)];
        r = norm(center-p1);
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