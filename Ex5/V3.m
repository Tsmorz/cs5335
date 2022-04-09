%% Tony Smoragiewicz
% V3 - RANSAC

close all
clear all

format short
format compact
w = warning ('off','all');

file = 'ex5_data/ptcloud.mat';
[xyz, rgb] = loadScan(file);
xyz = unwrap(xyz);
original = xyz;

% inliers = [mod(idx, r), ceil(idx/r)];


%% Find image planes
input('Find walls in the point cloud.')
figure(1)
view(-45, 30)
axis('equal')
hold on
plot3(xyz(:,1), xyz(:,2), xyz(:,3), 'b.')


e = [0.70, 0.70, 0.70];
s = 'plane';
threshold = 0.01;
% search for three planes
for i = 1:3
        [inliers, model] = RANSAC(xyz, e(i), s, threshold);
        disp('The plane contains the point:')
        disp(model(2,:))
        disp('The normal vector is:')
        disp(model(1,:))
        disp(' ')

        plane = xyz(inliers, :);
        plot3(plane(:,1), plane(:,2), plane(:,3), '.', 'Color', rand([1,3]) )
%         quiver3(xyz0(1), xyz0(2), xyz0(3), 0.15*n(1), 0.15*n(2), 0.15*n(3), 'k')
%         quiver3(xyz0(1), xyz0(2), xyz0(3), -0.15*n(1), -0.15*n(2), -0.15*n(3), 'k')

        % remove plane from data set
        xyz(inliers, :) = NaN;
end


%% Find the sphere
input('Find the ball in the point cloud.')
e = 0.70;
s = 'sphere';
threshold = 0.005;
[inliers, model] = RANSAC(xyz, e, s, threshold);
plot3(xyz(inliers,1), xyz(inliers,2), xyz(inliers,3), 'c.')
xyz(inliers, :) = NaN;

disp('The center is at:')
disp(model(2:4))
disp('The radius is:')
disp(model(1))
disp(' ')

[X, Y, Z] = sphere;
X2 = X*model(1);
Y2 = Y*model(1);
Z2 = Z*model(1);
surf(X2+model(2), Y2+model(3), Z2+model(4))

%% Find the wipes
input('Find the cylinder in the point cloud.')
e = 0.7;
s = 'tube';
threshold = 0.005;
[inliers, model] = RANSAC(xyz, e, s, threshold);
plot3(xyz(inliers,1), xyz(inliers,2), xyz(inliers,3), 'k.')
xyz(inliers, :) = NaN;
[X,Y,Z] = cylinder(model(3,1));
surf(X+model(1,1), Y+model(1,2), 0.3*Z, 'FaceColor', 'y')%+model(1,3))

disp('The center is at:')
disp(model(1,:))
disp('The normal vector is:')
disp(model(2,:))
disp('The radius is:')
disp(model(3,1))


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
        %quiver3(point(1), point(2), point(3), 0.15*normal(1), 0.15*normal(2), 0.15*normal(3), 'g')
        %quiver3(point(1), point(2), point(3), -0.15*normal(1), -0.15*normal(2), -0.15*normal(3), 'g')
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
                                inliers = and(outer, not(inner));
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
        1 - num / realPoints
        i/N
end

%% Load model
function [xyz, rgb] = loadScan(file)

        % Load Bunny
        data = load(file);
        rgb = data.ptcloud_rgb;
        xyz = data.ptcloud_xyz;
       
end
