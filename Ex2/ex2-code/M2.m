% Input: robot -> A 4-DOF robot encoded as a SerialLink object
%        q_min -> 1x4 vector of minimum angle for each joint
%        q_max -> 1x4 vector of maximum angle for each joint
%        num_samples -> Integer denoting number of samples in PRM
%        num_neighbors -> Integer denoting number of closest neighbors to
%                         consider in PRM
%        link_radius -> Scalar denoting radius of each robot link's
%                       cylindrical body
%        sphere_centers -> Nx3 matrix containing the centers of N spherical
%                          obstacles
%        sphere_radii -> Nx1 vector containing the radii of N spherical
%                        obstacles
% Output: samples -> num_samples x 4 matrix, sampled configurations in the
%                    roadmap (vertices)
%         adjacency -> num_samples x num_samples matrix, the weighted
%                      adjacency matrix denoting edges between roadmap
%                      vertices. adjacency(i,j) == 0 if there is no edge
%                      between vertex i and j; otherwise, its value is the
%                      weight (distance) between the two vertices. For an
%                      undirected graph, the adjacency matrix should be
%                      symmetric: adjacency(i,j) == adjacency(j,i)

function [samples, adjacency] = M2(robot, q_min, q_max, num_samples, num_neighbors, link_radius, sphere_centers, sphere_radii)

        % Find collision free samples
        samples = zeros([num_samples, length(q_min)]);
        i = 1;
        while i <= num_samples

                % Try a random sample
                q = M1(q_min, q_max, 1);
                crash = check_collision(robot, q, link_radius, sphere_centers, sphere_radii);

                % Add to samples if no crash
                if ~crash
                        samples(i,:) = q;
                        i = i + 1;
                end
        end

        % Find nearest neighbors
        adjacency = zeros(num_samples);
        for i = 2:num_samples
                q_start = samples(i,:);

                % Find num_neighbors closest nodes
                dist = sum( (samples(1:i-1,:) - q_start).^2, 2);
                [cost, idx] = mink(dist,num_neighbors);

                % Check for collision along edge
                for j = 1:length(idx)
                        q_end = samples(idx(j),:);
                        in_collision = check_edge(robot, q_start, q_end, link_radius, sphere_centers, sphere_radii);

                        if ~in_collision
                                adjacency(i,idx(j)) = cost(j);
                        end
                end 
        end

        % Symmetrize the adjacency matrix
        adjacency = adjacency + adjacency';

end