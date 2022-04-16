%% Tony Smoragiewicz
% V3 - RANSAC
function [] = V3(model, rgb)
        format short
        format compact
        warning ('off','all');
        
        xyz = model;
        xyz = unwrap(xyz);

        [r, c, ch] = size(rgb);
        

        %% Find surface normal
        fig = figure(1);

        while true
                clf;
                view(-45, 30)
                title('Point Cloud Segementation')
                axis('equal')
                movegui(fig, 'northwest')
                hold on

                plot3(xyz(:,1), xyz(:,2), xyz(:,3), 'c.')

                p = randPoint(xyz);
                radius = 0.02;
                [n, ~, points] = eigenNorm(p, radius, xyz);

                factor = 0.15;
                quiver3(p(1), p(2), p(3), factor*n(1), factor*n(2), factor*n(3), 'r')
                quiver3(p(1), p(2), p(3), -factor*n(1), -factor*n(2), -factor*n(3), 'r')
                plot3(points(:,1), points(:,2), points(:,3), 'k.')

                x = input('Enter to run again, any key + Enter to continue.');
                if isempty(x)
                        break
                end
        end
     
        
        %% Find image planes
        input('Find walls in the point cloud.')
        clf;
        fig = figure(1);
        view(-45, 30)
        title('Point Cloud Segementation')
        axis('equal')
        movegui(fig, 'northwest')
        hold on
        
        e = [0.70, 0.70, 0.70];
        s = 'plane';
        threshold = 0.01;
        % search for three planes
        for i = 1:3
                [inliers, model] = RANSAC(xyz, e(i), s, threshold);
                inliersImg = [mod(inliers, r), ceil(inliers/r)];

                disp('The plane contains the point:')
                p = model(2,:);
                disp(p)
                disp('The normal vector is:')
                n = model(1,:);
                disp(n)
                disp(' ')
        
                plane = xyz(inliers, :);

                color = rand([1,3]);
                plot3(plane(:,1), plane(:,2), plane(:,3), '.', 'Color', color )
                factor = 0.2;
                quiver3(p(1), p(2), p(3), factor*n(1), factor*n(2), factor*n(3), 'k')
                quiver3(p(1), p(2), p(3), -factor*n(1), -factor*n(2), -factor*n(3), 'k')

                % remove plane from data set
                xyz(inliers, :) = NaN;

                for j = 1:length(inliersImg)
                        for k = 1:3
                                rgb(inliersImg(j,1), inliersImg(j,2), k) = 255*color(k);
                        end
                end
        end

        figure(2)
        hold on
        title('Image Segmentation')
        imshow(rgb)
        

        %% Find the sphere
        input('Find the ball in the point cloud.')
        e = 0.70;
        s = 'sphere';
        threshold = 0.005;
        [inliers, model] = RANSAC(xyz, e, s, threshold);
        size(inliers)
        figure(1);
        hold on
        plot3(xyz(inliers,1), xyz(inliers,2), xyz(inliers,3), 'c.')
        xyz(inliers, :) = NaN;
        
        [X, Y, Z] = sphere;
        X2 = X*model(1);
        Y2 = Y*model(1);
        Z2 = Z*model(1);
        surf(X2+model(2), Y2+model(3), Z2+model(4))
                
        disp('The center is at:')
        disp(model(2:4))
        disp('The radius is:')
        disp(model(1))
        disp(' ')
        

        figure(2)
        inliersImg = [mod(inliers, r), ceil(inliers/r)];
        size(inliersImg)
        color = [0, 1, 1];
        for j = 1:length(inliersImg)
                for k = 1:3
                        u = inliersImg(j,1);
                        v = inliersImg(j,2);
                        rgb(u, v, k) = 255*color(k);
                end
        end
        imshow(rgb)

        %% Find the wipes
        input('Find the cylinder in the point cloud.')
        e = 0.7;
        s = 'tube';
        threshold = 0.005;
        [inliers, model] = RANSAC(xyz, e, s, threshold);
        figure(1);
        hold on
        plot3(xyz(inliers,1), xyz(inliers,2), xyz(inliers,3), 'k.')
        xyz(inliers, :) = NaN;
        [X,Y,Z] = cylinder(model(3,1));
        surf(X+model(1,1), Y+model(1,2), 0.3*Z, 'FaceColor', 'y')
        
        disp('The center is at:')
        disp(model(1,:))
        disp('The normal vector is:')
        disp(model(2,:))
        disp('The radius is:')
        disp(model(3,1))

end
