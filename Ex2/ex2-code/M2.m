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

        % Initial sampling (may contain crashes)
        qs = M1(q_min, q_max, num_samples);

        samples = qs;
        sz = [num_samples, num_samples];
        adjacency = zeros(sz);
        pos_xyz = ones([num_samples, 3]);

        % Loop through num_samples
        for i = 1:num_samples
                q = qs(i,:);
                [num_spheres, ~] = size(sphere_centers);
                
                % Loop through num_spheres
                for j = 1:num_spheres
                        sphere_center = sphere_centers(j,:);
                        sphere_radius = sphere_radii(j);
                        
                        % Check for a crash
                        crash = check_collision(robot, q, link_radius, sphere_center, sphere_radius);
                        while crash == 1
                                % Try new q if crash occured
                                q = M1(q_min, q_max, 1);
                                crash = check_collision(robot, q, link_radius, sphere_center, sphere_radius);
                        end
                        samples(i,:) = q; % save new q to list
                        pos_xyz(i,:) = robot.fkine(samples(i,:)).t;
                end
        end

        % Find nearest neighbors
        for i = 2:num_samples
                
                % Find num_neighbors closest nodes
                dist_pos = sum( (pos_xyz(1:i-1,:) - pos_xyz(i,:)).^2, 2);
                dist_qs = sum( (samples(1:i-1,:) - samples(i,:)).^2, 2);
                A = [1, 2];
                dist = A(1)*dist_qs + A(2)*dist_pos;
                [short_path, idx] = mink(dist,num_neighbors);

                for j = 1:length(idx)
                        q_start = samples(i,:);
                        q_end = samples(idx(j),:);

                        in_collision = check_edge(robot, q_start, q_end, link_radius, sphere_centers, sphere_radii);

                        if in_collision == 0
                                adjacency(i,idx(j)) = short_path(j);
                        end
                end 
        end

        % Symmetrize the matrix
        adjacency = adjacency + adjacency';

end