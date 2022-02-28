% Input: robot -> A 4-DOF robot encoded as a SerialLink object
%        q_min -> 1x4 vector of minimum angle for each joint
%        q_max -> 1x4 vector of maximum angle for each joint
%        q_start -> 1x4 vector denoting the start configuration
%        q_goal -> 1x4 vector denoting the goal configuration
%        link_radius -> Scalar denoting radius of each robot link's
%                       cylindrical body
%        sphere_centers -> Nx3 matrix containing the centers of N spherical
%                          obstacles
%        sphere_radii -> Nx1 vector containing the radii of N spherical
%                        obstacles
% Output: path -> Nx4 matrix containing a collision-free path between
%                 q_start and q_goal, if a path is found. The first row
%                 should be q_start, the final row should be q_goal.
%         path_found -> Boolean denoting whether a path was found

function [path, path_found] = M4(robot, q_min, q_max, q_start, q_goal, link_radius, sphere_centers, sphere_radii)
        
        % Create the graph
        possible_path = q_start;
        vert = 1;
        edge = [];
        G = digraph(vert,edge);

        % Rapidly Exploring Random Trees
        alpha = 0.2;
        beta = 0.5;
        i =  2;
        path_found = 0;
        while i<500
                % Find a new configuration
                if rand(1) < beta
                        % goal
                        q_target = q_goal;
                else
                        % random
                        q_target = M1(q_min, q_max, 1);
                end

                % Find the closest node
                dist = sqrt(sum( (possible_path - q_target).^2, 2));
                [weight, q_near_idx] = min(dist);
                q_near = possible_path(q_near_idx,:);

                % Step q_new towards target
                q_new = q_near + alpha * (q_target - q_near) / norm(q_target - q_near);
                q_new(3) = 0; % q(3) has no range of motion

                % Check if new node is in bounds
                in_bounds = (all(q_new >= q_min) && all(q_new <= q_max));
                if in_bounds
                        % Check for collision
                        in_collision = check_edge(robot, q_near, q_new, link_radius, sphere_centers, sphere_radii);

                        % Add configuration to path if not in collision
                        if ~in_collision
                                G = addedge(G, i, q_near_idx, weight);
                                possible_path(end+1,:) = q_new;
                                i = i + 1;
        
                                % break once a node has a clear path to end
                                xyz = norm( robot.fkine(q_goal).t - robot.fkine(q_new).t );
                                if xyz<=0.4
                                        in_collision = check_edge(robot, q_goal, q_new, link_radius, sphere_centers, sphere_radii,50);
                                        if and(~in_collision, xyz<0.2)
                                                possible_path(end+1,:) = q_goal;
                                                path_found = 1;
                                                break
                                        end
                                end
                        end
                end
        end

        % Descend tree to find path
        edges = G.Edges.EndNodes;
        [nodes, ~] = size(G.Nodes);
        path = q_goal;
        while nodes ~= 1
                path(end+1,:) = possible_path(nodes,:);
                nodes = edges(nodes-1,2);
        end

        % Add starting position to path
        path(end+1,:) = q_start;
        path = flipud(path);

        % Plot EF path
        marker = 'r-';
        plot_path(path, robot, marker);
        
end