%% Tony Smoragiewicz
% V3 - RANSAC

close all
format short
format compact
w = warning ('off','all');

file = 'ex5_data/ptcloud.mat';
[xyz, rgb] = loadScan(file);
xyz = unwrap(xyz);

% inliers = [mod(idx, r), ceil(idx/r)];


%% Find image planes
figure(2)
subplot(2,1,1)
view(-45, 30)
hold on
plot3(xyz(:,1), xyz(:,2), xyz(:,3), 'b.')
axis('equal')

subplot(2,1,2)
view(-45, 30)
axis('equal')
hold on

e = [0.65, 0.65, 0.65];
s = 3;
threshold = 0.01;
% search for three planes
for i = 1:3
        sample = 0.008;
        [inliers, ~] = RANSAC(xyz, e(i), s, threshold, sample);

        plane = xyz(inliers, :);
        plot3(plane(:,1), plane(:,2), plane(:,3), '.', 'Color', rand([1,3]) )

        % remove plane from data set
        xyz(inliers, :) = NaN;
end
plot3(xyz(:,1), xyz(:,2), xyz(:,3), 'b.')



%% Sort k-Objects in scene
% [num, idx] = numObjects(xyz);
% for i = 1:num
%         loc =  find(idx==i)';
%         plot3(xyz(loc,1), xyz(loc,2), xyz(loc,3), '.', 'Color', rand([1,3]))
% end


%% Find the sphere
e = 0.95;
s = 1;
threshold = 0.005;
sample = 1;
[~, model] = RANSAC(xyz, e, s, threshold, sample);

[X, Y, Z] = sphere;
X2 = X*model(1);
Y2 = Y*model(1);
Z2 = Z*model(1);
surf(X2+model(2), Y2+model(3), Z2+model(4))


%% Find the wipes
e = 0.95;
s = 2;
threshold = 0.0051;
sample = 1;
[inliers, model] = RANSAC(xyz, e, s, threshold, sample);


%% Random u,v coordinate with real xyz value
%       u,v must be one radius away from edge
function [point] = randPoint(data)

        [r, ~] = size(data);
        point = data(randi(r), :);

        while isnan(point)
                point = data(randi(r), :);
        end

end

%% Surface Normal from point
function [normal, ratio] = eigenNorm(point, radius, xyz)

        dist = sqrt( sum( (xyz-point).^2, 2 ) );
        points = xyz(dist<radius, :);

        mu = mean(points, 1, 'omitnan');
        pdiff = points - mu;

        Sigma = pdiff' * pdiff;
        [vec, lambda] =eig(Sigma);

        lambda = [lambda(1,1), lambda(2,2), lambda(3,3)];
        idx = find(lambda == min(lambda));
        normal = vec(:, idx)';

        ratio = min(lambda) / max(lambda);
        %plot3(points(:,1), points(:,2), points(:,3), 'g.')
        quiver3(point(1), point(2), point(3), 0.15*normal(1), 0.15*normal(2), 0.15*normal(3), 'g')
        quiver3(point(1), point(2), point(3), -0.15*normal(1), -0.15*normal(2), -0.15*normal(3), 'g')
end


%% Identify number of objects in scene
function [num, idx] = numObjects(data)

        maxNum = 10;
        value = zeros([maxNum-1,1]);
        for K = 2:maxNum
                [~, C] = kmeans(data, K);
        
                dist = zeros([K-1,1]);
                for i = 1:K
                        tmp = zeros([K-1,1]);
                        for j = 1:K
                                tmp(j) = norm(C(i,:) - C(j,:));
                        end
                        tmp(i) = max(tmp);
                        dist(i) = min(tmp);
                end
                value(K-1) = K^2*min(dist)^3;
        end
        
        idx = find(value == max(value));
        K = idx+1;
        [idx, C] = kmeans(data, K);
        for i = 1:K
                plot3(C(i,1), C(i,2), C(i,3), 'c*')
        end
       
        num = K;
end


%% Unwrap sensor matrix to a row
function row_vec = unwrap(data)
        [r, c, ~] = size(data);
        x = data(:,:, 1);
        x = reshape(x, [r*c, 1]);
        
        y = data(:,:, 2);
        y = reshape(y, [r*c, 1]);
        
        z = data(:,:, 3);
        z = reshape(z, [r*c, 1]);

        row_vec = [x, y, z];

end


%% Alternate surface normal given 3 points
function [normal, xyz] = surfaceNormal(xyz)
        U = xyz(2,:) - xyz(1,:);
        V = xyz(3,:) - xyz(1,:);
        normal = cross(U,V) / norm(cross(U,V));
        xyz = mean(xyz, 1);
end

%% RANSAC
% inliers = RANSAC(data, e, s, threshold):
% e - portion outliers
% s - num of points in sample
function [inliers, model] = RANSAC(data, e, s, threshold, sample)

        % Add a column of 1's to the end
        [r, c] = size(data);
        realPoints = sum(~isnan(data(:,1)));

        % num of points to compare against from data set
        subset = round(sample*realPoints);
        
        % number of samples needed
        p = 0.99;
        N = round( log(1-p) / log(1-(1-e)^s) );
        
        % Stop search when this number has been reached
        T = sample * (1-e)*realPoints;
        
        % Number of points within threshold distance
        num = 0;

        i = 1;
        while i < N
                % try a random set points that fits the model
                xyz = zeros([s, c]);
                j = 1;
                while j <= s
                        point = randPoint(data);
                        if ~isnan(point)
                                xyz(j,:) = point;
                                j = j + 1;
                        end
                end
                if s == 1
                        % find surface normal
                        [n, ~] = eigenNorm(point, 0.01, data);

                        r_min = 0.05;
                        r_max = 0.10;
                        offset = r_max - r_min;
                        noise = 2*offset*rand(1) - 2*offset/2;
                        radius = (r_max + r_min)/2 + noise;
                        center = point + radius*n;

                        dist = sqrt( sum( (data-center).^2, 2 ) );
                        outer = dist <= radius + threshold;
                        inner = dist <= radius - threshold;

                        if sum(and(outer, not(inner))) > num
                                num = sum(and(outer, not(inner)));
                                xyz0 = center;
                                model = [radius, center];
                                points = and(outer, not(inner));
                        end

                elseif s == 2
                        [n1, ~] = eigenNorm(xyz(1,:), 0.01, data);
                        [n2, ~] = eigenNorm(xyz(2,:), 0.01, data);
                        vec = cross(n1,n2);

                        if abs(max(c)) > 0.9
                                % pipe along vector
                                r_min = 0.05;
                                r_max = 0.10;
                                offset = r_max - r_min;
                                noise = 2*offset*rand(1) - 2*offset/2;
                                radius = (r_max + r_min)/2 + noise;
                        end
                        
                else
                        % find the surface normal of the plane
                        [temp_n, temp_xyz0] = surfaceNormal(xyz);
        
                        temp_xyz = zeros([subset, 4]);
                        for k = 1:subset
                                point = NaN;
                                while isnan(point)
                                        % try a random point
                                        idx = randi(r);
                                        point = data(idx, :);
                                end
                                temp_xyz(k,:) = cat(2, idx, point);
                        end
                        dist = abs( (temp_xyz(:,2:end) - temp_xyz0) * temp_n');
                        temp_num = length(find(dist<=threshold));
        
                        % save new data
                        if temp_num > num
                                num = temp_num;
                                xyz0 = temp_xyz0;
                                n = temp_n;
                                if num >= T
                                        break
                                end
                        end
                end

                % increment counter
                i = i + 1;
        end
        if s == 1
                plot3(data(points,1), data(points,2), data(points,3), 'c.')
                plot3(model(2), model(3), model(4), 'r.')
                sum(points)/length(data)
                inliers = points;
        elseif s == 2

        else
                num / subset
        
                % calculate distance to all points
                distance = abs( (data - xyz0) * n' );
        
                % Double threshold and identify points on plane
                inliers = find(distance <= 1.5*threshold);
        
                % plot the best points
                quiver3(xyz0(1), xyz0(2), xyz0(3), 0.15*n(1), 0.15*n(2), 0.15*n(3), 'r')
                quiver3(xyz0(1), xyz0(2), xyz0(3), -0.15*n(1), -0.15*n(2), -0.15*n(3), 'r')
                model = n;
        end

end

%% Load model
function [xyz, rgb] = loadScan(file)

        % Load Bunny
        data = load(file);
        rgb = data.ptcloud_rgb;
        xyz = data.ptcloud_xyz;
       
end
