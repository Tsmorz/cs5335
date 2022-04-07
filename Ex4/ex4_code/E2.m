% Input: odo -> 2xT matrix containing odometry readings for T time steps
%        zind -> 1xT vector containing the observed landmark index for
%                T time steps; index is 0 if no landmark observed
%        z -> 1xT cell array containing the (range, bearing) observation
%             for T time steps; z{t} is empty if no observation at time t
%        W -> 2x2 matrix denoting the sensing noise in (range, bearing)
%        x0 -> 3x1 vector denoting the known initial vehicle state
% Output: x_est -> 1xT cell array containing the map state mean
%                  for T time steps (i.e., x_est{t} is a (2M)x1 vector,
%                  where M is the number of landmarks observed by time t)
%         P_est -> 1xT cell array containing the vehicle state covariance
%                  for T time steps (i.e., P_est{t} is a (2M)x(2M) matrix,
%                  where M is the number of landmarks observed by time t)
%         indices -> Mx1 vector containing the landmark index corresponding
%                    to the entries in the state vector, where M is the
%                    number of landmarks observed by the final time step T)
%                    For example, if indices is [15; 4], then the first two
%                    rows of x_est and P_est correspond to landmark 15,
%                    and the next two rows correspond to landmark 4, etc.

function [x_est, P_est, indices] = E2(odo, zind, z, W, x0)

%         figure(1)
%         hold on

        % initialize outputs
        x_est = {};
        P_est = {};
        indices = [];

        % initial location
        vehicle_pos = x0;

        % loop through all odometry readings
        for i = 1:length(odo)
                %% Track vehicle location

                % previous position
                x = vehicle_pos(1, end);
                y = vehicle_pos(2, end);
                heading = vehicle_pos(3, end);

%                 % Plotting vehicle location
%                 plot(x, y, 'k.')

                % position update
                dx = odo(1,i);
                dtheta = odo(2,i);
                vehicle_pos(:, end+1) = [x + dx*cos(heading);
                                                        y + dx*sin(heading);
                                                        angdiff(heading, -dtheta)];

                % position during measurement
                x = vehicle_pos(1, end);
                y = vehicle_pos(2, end);
                heading = vehicle_pos(3, end);

                %% Landmark was detected
                if zind(i)>0

                        % Sensor readings
                        measurement = z{i};
                        r = measurement(1);
                        bearing = measurement(2);

                        % Predict landmark location
                        x_pred = [x + r * cos(angdiff(heading, -bearing));
                                        y + r * sin(angdiff(heading, -bearing))];

%                         % plotting landmark measurement
%                         plot(x_pred(1), x_pred(2), 'r.')
%                         pause(0.01)

                        %% New landmark
                        [idx, ~] = find(indices == zind(i));
                        if isempty(idx)
                                indices = vertcat(indices, zind(i));

                                % existing sensor readings
                                if length(x_est) >= 1
                                        x_old = x_est{end};
                                % first sensor reading
                                else
                                        x_old = cell2mat(x_est);
                                end

                                new = vertcat(x_old, x_pred);
                                x_est(end+1) = mat2cell(new, length(x_old)+2);

                                %% Insertian Jacobian
                                % Covariance Update
                                ang = angdiff(heading, -bearing);
                                Gz = [cos(ang), -r*sin(ang);
                                           sin(ang),  r*cos(ang)];

                                n = length(new);
                                if n > 2
                                        Yz_upper = horzcat(eye(n-2), zeros([n-2, 2]));
                                        Yz_lower = horzcat(zeros([2, n-2]), Gz);
                                        Yz = vertcat(Yz_upper, Yz_lower);
                                else
                                        Yz = Gz;
                                end

                                if isempty(P_est)
                                        P = W;
                                else
                                        n = length(P_est{end});
                                        P_upper = horzcat(P_est{end}, zeros([n, 2]));
                                        P_lower = horzcat(zeros([2, n]), W);
                                        P = vertcat(P_upper, P_lower);
                                end
                                P_est(end+1) = mat2cell(Yz*P*Yz', length(Yz));

                        %% Existing landmark
                        else
                                % landmark location
                                xy = x_est{end};
                                [num_LM, ~] = size(xy);
                                land_x = xy(2*idx-1);
                                land_y = xy(2*idx);
                                
                                % expected measurement
                                xdiff = land_x - x;
                                ydiff = land_y - y;
                                r = sqrt(sum(xdiff^2 + ydiff^2));
                                ang = angdiff(atan2(ydiff, xdiff), heading);

                                % Jacobian of (r, β) with respect to landmark position (xi , yi)
                                Hp = [xdiff/r,     ydiff/r;
                                         -ydiff/r^2, xdiff/r^2];
                                Hx = zeros([2, num_LM]);
                                Hx(:, 2*idx-1:2*idx) = Hp;

                                %% Update Step
                                % innovation
                                z_sharp = z{i};
                                nu = [z_sharp(1) - r;
                                        angdiff(z_sharp(2), ang)];

                                % kalman gain
                                Hw = eye(2);
                                S = Hx * P_est{end} * Hx' + Hw * W * Hw';
                                K = P_est{end} * Hx' / S;

                                % posterior
                                l = length(x_est{end});
                                x_est(end) = mat2cell(x_est{end} + K*nu, l);
                                P_est(end) = mat2cell(P_est{end} - K*Hx*P_est{end}, l, l);

                        end % landmark update

                end % landmark detected

        end % loop through odometry

end