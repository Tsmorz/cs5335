% Input: robot -> A 4-DOF robot encoded as a SerialLink object
%        path -> Nx4 matrix containing a collision-free path between
%                q_start and q_goal
%        link_radius -> Scalar denoting radius of each robot link's
%                       cylindrical body
%        sphere_centers -> Nx3 matrix containing the centers of N spherical
%                          obstacles
%        sphere_radii -> Nx1 vector containing the radii of N spherical
%                        obstacles
% Output: smoothed_path -> Nx4 matrix containing a smoothed version of the
%                          input path, where some unnecessary intermediate
%                          waypoints may have been removed

function smoothed_path = M5(robot, path, link_radius, sphere_centers, sphere_radii)
        
        smoothed_path = path;
        n = length(smoothed_path)-1;
        i = 2;
        while i < n
                q_start = smoothed_path(i-1,:);
                q_end = smoothed_path(i+1,:);
                dist = norm( robot.fkine(q_start).t - robot.fkine(q_end).t );
                in_collision = check_edge(robot, q_start, q_end, link_radius, sphere_centers, sphere_radii,20);
                if and(~in_collision,dist<0.3)
                        smoothed_path(i,:) = [];
                        n = n - 1;
                        i = i - 1;
                end
                i = i + 1;
        end

        plot_path(smoothed_path, robot, 'k-');

end