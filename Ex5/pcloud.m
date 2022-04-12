close all

dales = load('dalesLIDAR.mat', '-mat');
points = dales.Untitled;
points = table2array(points);

seg = 1:45970;
section = points(seg,:);
point = randPoint(section);

point1 = [288.2, 312.7, 37.5];
point2 = [288.2, 311.7, 37.5];

pose = eye(4);

% figure(1)
% axis('equal')
% hold on
% subplot(2, 2, [1 3])
% scatter3(points(seg,1), points(seg,2), points(seg,3), 'b.')

radius = 10;
[normal, ratio, points1] = eigenNorm(point1, radius, section);
[normal, ratio, points2] = eigenNorm(point2, radius, section);

T = ICP(points1', (points2 + [0, 1, 4])')

% figure(1)
% hold on
% scatter3(point1(1), point1(2), point1(3), 'r*')
% scatter3(point2(1), point2(2), point2(3), 'r*')
% 
% pose = T*pose;
% scatter3(pose(1,4), pose(2,4), pose(3,4), 'k*')
