% Input: robot -> A 2-DOF robot encapsulated in a MATLAB cell with fields:
%                 robot.link1, robot.link2, robot.pivot1, robot.pivot2
%                 See description in hw2_cspace and demo code in C1.
%        obstacles -> 1xN vector of polyshape objects describing N 2-D
%                     polygonal obstacles
%        q_grid -> 1xN vector of angles between 0 and 2*pi, discretizing
%                  each dimension of configuration space
% Output: cspace -> NxN matrix: cspace(i,j)
%                   == 1 if [q_grid(i); q_grid(j)] is in collision,
%                   == 0 otherwise

function cspace = C22(robot, obstacles, q_grid)
        
        % Initialize cspace grid
        cspace = zeros(length(q_grid));

        % Loop through all angles in q1
        for i = 1:length(q_grid)

                % Find new orientation after rotation
                q = [q_grid(i); 0];
                [poly1, ~, ~, ~] = q2poly(robot,q);
                
                % Loop through all obstacles
                for k = 1:length(obstacles)
                        crash = intersect(poly1,obstacles(k));

                        % Dont check Link2 if Link1 collided
                        if crash.NumRegions ~= 0
                                cspace(i,:) = 1;

                        % Check Link 2
                        else
                                for j = 1:length(q_grid)
                                        q = [q_grid(i); q_grid(j)];
                
                                        % Find new orientation after rotation
                                        [~, poly2, ~, ~] = q2poly(robot,q);
                                        crash = intersect(poly2,obstacles(k));
                
                                        % Check for intersection of links and obstacles
                                        if crash.NumRegions ~= 0
                                                cspace(i,j) = 1;
                                        end

                                end

                        end

                end

        end
        
end