%%Segment Detection
% testgetDist()


function [endpoints,new_data] = segment_ransac_c4(x,y,debug)
    min_seg_length = 10;
%     %load playpensample.mat
%     [ctheta, cr] = cleanData(theta,r);
%     [x,y] = polar2cart(deg2rad(ctheta),cr);
    if debug
        figure; hold on
        plot(x,y,'ks')
        plot(0,0,'kO')
        axis('equal')
    end
%     legendata = ["lidar data","neato location"];
    data = [x y];
    endpoints = [];
    for i = 1:35
        if length(data) < 4
            fprintf("Ended after %d iterations.\n",i-1)
            break
        end
        [p1,p2,in,out] = robustLineFit(data(:,1),data(:,2),0.2,floor(length(data*100)), 0.7);
        %quiver(p1(1),p1(2),p2(1)-p1(1),p2(2)-p1(2),'LineWidth',2)
        if ~isempty(in)
            if debug
                plot(in(:,1),in(:,2),'*')
                title('long segments only')
            end
            endpoints = [endpoints; p1, p2, length(in)];
        end
        data = out;
        %legendata = [legendata, sprintf("round %d line",i)];
        %legendata = [legendata, sprintf("round %d inliers",i)];
        
    end
    new_data = data;
%     %legend(legendata)
%     axis equal
%     xlabel 'x pos (meters)'
%     ylabel 'y pos (meters)'
%     title 'RANSAC Implementation'

    function testgetDist()
        p1 = [0 0];
        p2 = [1 1];
        p3 = [0 1];
        getDist(p1,p2,p3)  % should be sqrt(2)/2

        p1 = [3 3];
        p2 = [3 -3];
        p3 = [2 0];
        getDist(p1,p2,p3)  % should be 1

        p1 = [0 0];
        p2 = [1 1];
        p3 = [0 1; 1 0; 2 2];
        getDist(p1,p2,p3)
    end

    function [p1,p2,inliers,outliers] = robustLineFit(x, y, d, n, segment_length)
    %ROBUSTLINEFIT  Fit lines to scanner data with RANSAC
        data = [x y];
        choices = randsample(length(data),2*n, 'true');
        for i = 2:2:length(choices)
            p1 = data(choices(i-1),:);
            p2 = data(choices(i),:);
            D = getDist(p1,p2,data);  % get distances of all data points to line
            results.inliers(i / 2) = sum(D < d);
            results.distances(:,i / 2) = D;
        end
        [sR, sI] = sort(results.inliers,'descend');
        sP2 = data(choices(sI*2),:);
        sP1 = data(choices(sI*2 - 1),:);
    %     figure; hold on
    %     plot(data(:,1),data(:,2),'ks')  % cleaned lidar data
    %     for i = 1:3
    %         quiver(sP1(i,1),sP1(i,2),sP2(i,1)-sP1(i,1),sP2(i,2)-sP1(i,2),'LineWidth',2)
    %     end
    %     legend('data','line 1','line 2','line 3')

        p1 = sP1(1,:);
        p2 = sP2(1,:);
        D = results.distances(:,sI(1));
        inliers = data(D < d,:);
        [in_elem, in_index] = sort(inliers(:,1),'descend');
        inliers = inliers(in_index, :);
        dists = sqrt(sum(diff(inliers).^2, 2));
    %     figure
    %     hold on
    %     
        if max(dists) >  segment_length
            inliers = inliers(1:find(dists > segment_length, 1), :);
        end
        outliers = setdiff(data, inliers, 'rows');
        
        p1 = inliers(1,:);
        p2 = inliers(end, :);
        hold on
        plot(inliers(:,1), inliers(:,2), 'bo')
        disp(inliers);
        disp(size(inliers,1) < min_seg_length);
        if size(inliers,1) < min_seg_length
            p1 = [];
            p2 = [];
            outliers = data;
            inliers = [];
        end
         

    end

    function dist = getDist(p1,p2,p3)
    % GETDIST  finds the perpendicular distance from p3 to the line
    % through p1 and p2.
    % p3 is a collection of x and y points of format [X Y]

        line_points = num2cell([p1 p2]);
        [x1,y1,x2,y2] = deal(line_points{:});
        x0 = p3(:,1);
        y0 = p3(:,2);
        dist = abs((y2-y1).*x0 - (x2-x1).*y0 + x2.*y1 - y2.*x1) / sqrt((y2-y1).^2 + (x2-x1).^2);
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
