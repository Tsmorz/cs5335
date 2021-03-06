% Input: odo_truth -> 2xT matrix containing true (non-noisy)
%                     odometry readings for T time steps
%        map -> Robotics toolbox Map object containing the known map
%               (known landmarks)
%        V -> 2x2 matrix denoting the process noise in (forward, angular)
%        W -> 2x2 matrix denoting the sensing noise in (range, bearing)
%        x0 -> 3x1 vector denoting the initial vehicle state mean
%        range -> Scalar denoting the maximum range of the sensor
%        fov -> 1x2 vector denoting the [min_bearing, max_bearing]
%               (field of view) that can be detected by the sensor
%        mode -> Character denoting sensing mode
%                'o': One visible landmark (if any) detected per time step
%                'a': All visible landmarks (if any) detected per time step
%                'f': False positives detections are added to observations
% Output: odo -> 2xT matrix containing noisy odometry readings for T time steps
%         zind -> 1xT cell array containing the observed landmark indices
%                 for T time steps; zind{t} is empty if no landmarks observed
%         z -> 1xT cell array containing the (range, bearing) observation
%              for T time steps; z{t} is empty if no observation at time t

function [odo, zind, z] = E5(odo_truth, map, V, W, x0, range, fov, mode)

        % create noisy odometry
        T = length(odo_truth);
        stdV = sqrtm(V);
        odo_noise = [stdV(1,1); stdV(2,2)] .* randn([2, T]);
        odo = odo_truth + odo_noise;

        % initialize outputs
        zind = {};
        z = {};

        % vehicle location
        pos_truth = zeros([3, T]);
        pos_truth(:,1) = x0;

        % landmark positions from map
        landmark_pos = map.map(:);
        landmark_pos = reshape(landmark_pos, [length(landmark_pos)/2, 2]);
        landmark_id = 1:length(landmark_pos);

%         figure(1)
%         hold on

        % loop through all odometry readings
        for i = 1:length(odo_truth)-1

                % update position
                [x ,y, heading] = new_pos(pos_truth(:,i), odo_truth(:,i));
                pos_truth(:, i+1) = [x; y; heading];
                
%                 plot(x, y, 'r.')

                %% Check landmark locations against sensor limits
                % distance from vehicle to landmark
                landmark_xy = landmark_pos - [x, y];
                landmark_dist = sqrt(sum((landmark_xy).^2, 2));

                % angle to landmark minus heading angle
                landmark_ang = atan2(landmark_xy(:,2), landmark_xy(:,1));
                landmark_ang = angdiff(landmark_ang, heading);

                % check if landmark is within range and fov
                [idx_range, ~] = find(landmark_dist<range);
                idx_fov = and(landmark_ang<max(fov), landmark_ang>min(fov));
                idx_bearing = landmark_id(idx_fov)';
                [idx, ~] = intersect(idx_range, idx_bearing);

                % landmark is within range and fov
                if ~isempty(idx)

                        for j = 1:length(idx)
                                x = landmark_pos(idx(j), 1);
                                y = landmark_pos(idx(j), 2);
%                                 plot(x, y, 'g*')
                        end

                        % z measurements + noise
                        z_range = landmark_dist(idx) + sqrt(W(1,1))*rand(1);
                        z_bearing = landmark_ang(idx) + sqrt(W(2,2))*rand(1);

                        %% Handle different modes
                        % one landmark visible
                        if mode == 'o'
                                % random landmark
                                n = randi(length(idx));
                                zind(end+1) = {idx(n)};
                                z(end+1) = {[z_range(n), z_bearing(n)]};
                                z{end};

                        % all landmarks visible
                        elseif mode == 'a'
                                zind(end+1) = {idx};
                                z(end+1) = {[z_range, z_bearing]};

                        % false positive
                        elseif mode == 'f'
                                disp(mode)

                        % wrong mode
                        else
                                disp(mode)
                        end

                % no measurement
                else
                        zind(end+1) = {[]};
                        z(end+1) = {[]};
%                         plot(landmark_pos(:,1), landmark_pos(:,2), 'k*')

                end


        end

end


% get vehicle state
% helper function for cleaner code
function [x, y, heading] = new_pos(pos, odo)
        x = pos(1);
        y = pos(2);
        heading = pos(3);

        dx = odo(1);
        dtheta = odo(2);

        x = x + dx*cos(heading);
        y = y + dx*sin(heading);
        heading = angdiff(heading, -dtheta);
end