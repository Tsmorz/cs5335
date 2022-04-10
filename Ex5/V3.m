%% Tony Smoragiewicz
% V3 - RANSAC
function [] = V3(model)
        format short
        format compact
        w = warning ('off','all');
        
        xyz = model;
        xyz = unwrap(xyz);
%         original = xyz;
        
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

end
