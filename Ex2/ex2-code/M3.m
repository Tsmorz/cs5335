% Input: robot -> A 4-DOF robot encoded as a SerialLink object
%        samples -> num_samples x 4 matrix, vertices in the roadmap
%        adjacency -> num_samples x num_samples matrix, the weighted
%                     adjacency matrix denoting edges in the roadmap
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

function [path, path_found] = M3(robot, samples, adjacency, q_start, q_goal, link_radius, sphere_centers, sphere_radii)
        
        % Find EF positions from samples
        X = zeros([length(samples),3]);
        for i = 1:length(samples)
                X(i,:) = robot.fkine(samples(i,:)).t;
        end
        
        % Create graph with adjacency matrix
        G = graph(adjacency);

        % Find closest starting and goal positions
        pos_start = robot.fkine(q_start).t';
        [~,idx_start] = min(sum((X-pos_start).^2, 2));
        pos_goal = robot.fkine(q_goal).t';
        [~,idx_goal] = min(sum((X-pos_goal).^2, 2));
        
        % Find a path
        path_idx = shortestpath(G, idx_start, idx_goal);
        if isempty(path_idx)
                path_found = 0;
        else
                % Start and end path with q_start and q_goal
                path = samples(path_idx,:);
                path = cat(1,q_start,path);
                path = cat(1,path,q_goal);

                for j = 1:length(path)-1
                        q_start = path(j,:);
                        q_end = path(j+1,:);
                        in_collision = check_edge(robot, q_start, q_end, link_radius, sphere_centers, sphere_radii);
                        if in_collision
                                disp('hit')
                        end
                end
                path_found = 1;
        end 

        % Plot samples and path
        plot_path(path,robot,'r-')

end