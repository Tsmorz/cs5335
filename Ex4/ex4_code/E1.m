% Input: odo -> 2xT matrix containing odometry readings for T time steps
%        zind -> 1xT vector containing the observed landmark index for
%                T time steps; index is 0 if no landmark observed
%        z -> 1xT cell array containing the (range, bearing) observation
%             for T time steps; z{t} is empty if no observation at time t
%        V -> 2x2 matrix denoting the process noise in (forward, angular)
%        W -> 2x2 matrix denoting the sensing noise in (range, bearing)
%        x0 -> 3x1 vector denoting the initial vehicle state mean
%        P0 -> 3x3 matrix denoting the initial vehicle state covariance
%        map -> Robotics toolbox Map object containing the known map
%               (known landmarks) for localization
% Output: x_est -> 1xT cell array containing the vehicle state mean
%                  for T time steps (i.e., x_est{t} is a 3x1 vector)
%         P_est -> 1xT cell array containing the vehicle state covariance
%                  for T time steps (i.e., P_est{t} is a 3x3 matrix)

function [x_est, P_est] = E1(odo, zind, z, V, W, x0, P0, map)

        % Initialize output vectors
        x_est = {x0};
        P_est = {P0};

        % Loop through all readings
        for i = 1:length(odo)
                %% Prediction Step
                x_current = x_est{end};
                x = x_current(1);
                y = x_current(2);
                heading = x_current(3);

                dx = odo(1, i);
                dtheta = odo(2, i);

                X_pred = [x + dx*cos(heading);
                                y + dx*sin(heading);
                                angdiff(heading, -dtheta)];
                Fx = [1, 0, -dx*sin(V(2,2));
                        0, 1, dx*cos(V(2,2));
                        0, 0, 1];
                Fv = [cos(V(2,2)), 0;
                        sin(V(2,2)), 0;
                        0, 1];

                P_pred = Fx*P_est{end}*Fx.' + Fv*V*Fv.';

                %% Innovation Step
                if ~isempty(z{:,i})
                        j = zind(i);
                        landmark_pos = map.map(:, j);

                        xdiff = landmark_pos(1) - X_pred(1);
                        ydiff = landmark_pos(2) - X_pred(2);
                        r = sqrt(sum(xdiff^2 + ydiff^2));
                        ang = angdiff(atan2(ydiff, xdiff) - heading);

                        Hx = [- xdiff/r, -ydiff/r, 0;
                                ydiff/r^2, -xdiff/r^2, -1];
                        Hw = eye(2);

                        % nu
                        z_sharp = z{i};
                        nu = [z_sharp(1) - r;
                                angdiff(z_sharp(2), ang)];

                        % kalman gain
                        S = Hx*P_pred*Hx.' + Hw*W*Hw.';
                        K = P_pred*Hx.' / S;

     
                %% Update Step
                        % update with sensor reading
                        x_est(end+1) = {X_pred + K*nu};
                        P_est(end+1) = {P_pred - K*Hx*P_pred};


                else
                        % update with only odometry
                        x_est(end+1) = {X_pred};
                        P_est(end+1) = {P_pred};
                end


        end

end