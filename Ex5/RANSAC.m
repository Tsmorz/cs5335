%% RANSAC
% inliers = RANSAC(data, e, s, threshold):
% e - portion outliers
% s - num of points in sample
function [inliers, model] = RANSAC(data, e, s, threshold)

        % # of rows with real entries
        realPoints = sum(~isnan(data(:,1)));

        % Stop search when this number has been reached
        T = (1-e)*realPoints;
        
        % number of samples needed
        p = 0.99;
        if strcmp(s, 'plane')
                variables = 3;
        elseif strcmp(s, 'sphere')
                variables = 2;
        else
                variables = 3;
        end

        N = round( log(1-p) / log(1-(1-e)^variables) );
        
        % Number of points within threshold distance
        num = 0;

        i = 1;
        while i < N
                % try a random set points that fits the model
                xyz = zeros([variables, 3]);
                j = 1;
                while j <= variables
                        point = randPoint(data);
                        xyz(j,:) = point;
                        j = j + 1;
                end
                if strcmp(s, 'sphere')
                        % find surface normal from random point
                        [n, ~] = eigenNorm(xyz(1,:), 0.01, data);

                        r_min = 0.05;
                        r_max = 0.10;
                        offset = r_max - r_min;
                        noise = offset*rand(1)/2;
                        radius = (r_max + r_min)/2 + noise;

                        % center of sphere is distance r from random point along
                        % normal vector
                        center = xyz(1,:) + radius*n;

                        % distance from center to all points
                        dist = sqrt( sum( (data-center).^2, 2 ) );

                        % points inside outer radius
                        outer = dist <= radius + threshold;

                        % points inside inner radius
                        inner = dist <= radius - threshold;

                        % number of points within the outer/inner shell
                        count = sum(and(outer, not(inner)));
                        if count > num
                                num = count;
                                model = [radius, center];
                                idx = and(outer, not(inner));
                                inliers = 1:length(idx);
                                inliers = inliers(idx);
                                inliers = inliers';
                        end

                elseif strcmp(s, 'tube')
                        [n1, ~] = eigenNorm(xyz(1,:), 0.01, data);
                        [n2, ~] = eigenNorm(xyz(2,:), 0.01, data);
                        n = cross(n1,n2);
                        n = n / norm(n);
    
                        r_min = 0.05;
                        r_max = 0.10;
                        offset = r_max - r_min;
                        noise = offset*rand(1)/2;
                        radius = (r_max + r_min)/2 + noise;

                        % center of sphere is distance r from random point along
                        % normal vector
                        point = xyz(1,:);
                        center = point + radius*n1;

                        % dist point on surface to center
                        r2c = data - center;
                        n_mat = repmat(n, [length(r2c),1]);
                        dist = cross(r2c', n_mat')';
                        dist = sqrt(sum(dist.^2,2));

                        % points inside shell of chosen radius
                        outer = dist <= radius + threshold;
                        inner = dist <= radius - threshold;
                        count = and(outer, not(inner));

                        if sum(count) > num
                                num = sum(count);
                                inliers = count;
                                model = [center; n; radius, 0, 0];
                        end

                else
                        % find the surface normal of the plane
                        [n, point] = surfaceNormal(xyz);
                        dist = abs( (data - point) * n');
                        count = length(find(dist<=threshold));
        
                        % save new data
                        if count > num
                                num = count;
                                model = [n; point];
                                inliers = find(dist <= 1.5*threshold);
                        end
                end

                if num >= T
                        break
                end
                % increment counter
                i = i + 1;
        end

end

%% Alternate surface normal given 3 points
function [normal, xyz] = surfaceNormal(xyz)
        U = xyz(2,:) - xyz(1,:);
        V = xyz(3,:) - xyz(1,:);
        normal = cross(U,V) / norm(cross(U,V));
        xyz = mean(xyz, 1);
end
