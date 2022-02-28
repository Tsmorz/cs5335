% Input: robot -> A 2-DOF robot encapsulated in a MATLAB cell with fields:
%                 robot.link1, robot.link2, robot.pivot1, robot.pivot2
%                 See description in hw2_cspace and demo code in C1.
%        obstacles -> 1xN vector of polyshape objects describing N 2-D
%                     polygonal obstacles
%        q_path -> Mx2 matrix containing a collision-free path from
%                  q_start to q_goal. Each row in q_path is a robot
%                  configuration. The first row should be q_start,
%                  the final row should be q_goal.
% Output: num_collisions -> Number of swept-volume collisions encountered
%                           between consecutive configurations in q_path

function num_collisions = C6(robot, obstacles, q_path)

        num_collisions = 0;
        for i = 1:length(q_path)-1
                for j = 1:length(obstacles)

                        % Find link locations
                        [poly1i, poly2i, ~, ~] = q2poly(robot,q_path(i,:));
                        [poly1ii, poly2ii, ~, ~] = q2poly(robot,q_path(i+1,:));
        
                        % Find swept shapes
                        X = [poly1i.Vertices(:,1); poly1ii.Vertices(:,1)];
                        Y = [poly1i.Vertices(:,2); poly1ii.Vertices(:,2)];
                        idx = convhull(X,Y);
                        sweep1 = polyshape([X(idx), Y(idx)]);

                        X = [poly2i.Vertices(:,1); poly2ii.Vertices(:,1)];
                        Y = [poly2i.Vertices(:,2); poly2ii.Vertices(:,2)];
                        idx = convhull(X,Y);
                        sweep2 = polyshape([X(idx), Y(idx)]);

                        % Check for a crash
                        crash1 = intersect(sweep1,obstacles(j));
                        crash2 = intersect(sweep2,obstacles(j));
                        if or(crash1.NumRegions > 0, crash2.NumRegions > 0)
                                C1(robot,q_path(i,:))
                                C1(robot,q_path(i+1,:))

                                plot(sweep1, 'FaceColor', 'r');
                                plot(sweep2, 'FaceColor', 'b');

                                num_collisions = num_collisions + 1;
                        end
                end
        end

end