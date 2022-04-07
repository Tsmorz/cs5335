% Input: odo -> 2xT matrix containing odometry readings for T time steps
%        zind -> 1xT vector containing the observed landmark index for
%                T time steps; index is 0 if no landmark observed
%        z -> 1xT cell array containing the (range, bearing) observation
%             for T time steps; z{t} is empty if no observation at time t
%        V -> 2x2 matrix denoting the process noise in (forward, angular)
%        W -> 2x2 matrix denoting the sensing noise in (range, bearing)
%        x0 -> 3x1 vector denoting the initial vehicle state mean
%        P0 -> 3x3 matrix denoting the initial vehicle state covariance
% Output: x_est -> 1xT cell array containing the map state mean
%                  for T time steps (i.e., x_est{t} is a (3+2M)x1 vector,
%                  where M is the number of landmarks observed by time t)
%         P_est -> 1xT cell array containing the vehicle state covariance
%                  for T time steps (i.e., P_est{t} is a (3+2M)x(3+2M) matrix,
%                  where M is the number of landmarks observed by time t)
%         indices -> Mx1 vector containing the landmark index corresponding
%                    to the entries in the state vector, where M is the
%                    number of landmarks observed by the final time step T)
%                    For example, if indices is [15; 4], then the first
%                    three rows of x_est and P_est correspond to the
%                    vehicle state, the next two correspond to landmark 15,
%                    and the next two rows correspond to landmark 4, etc.

function [x_est, P_est, indices] = E3(odo, zind, z, V, W, x0, P0)

        figure(1)
        hold on

        % initialize outputs
        x_est = {x0};
        P_est = {P0};
        indices = [];

        % loop through all odometry readings
        for i = 1:length(odo)-1


                %% Prediction Step
                x_pred = x_est{end};


                vehicle_pred = vehicle_pos(x_pred(1:3), odo(:,i));

                x_pred(1:3) = vehicle_pred;
                dx = odo(1,i);
                heading = vehicle_pred(3);

                % Process noise Jacobians
                M = length(x_pred);
                Fx = ones(M, M);
                Fx(1:3,1:3) = [1, 0, -dx*sin(heading);
                                       0, 1, dx*cos(heading);
                                       0, 0, 1];

                Fv = zeros(M, 2);
                Fv(1:3,1:2) = [cos(heading), 0;
                                       sin(heading), 0;
                                        0, 1];

                P_pred = Fx*P_est{end}*Fx.' + Fv*V*Fv.';

                % Landmark detected
                if zind(i)>0
                        measurement = z{i};
                        r = measurement(1);
                        bearing = measurement(2);
                        LM_pred = LM_pos(vehicle_pred, measurement);

                        %% New Landmark
                        [idx, ~] = find(indices == zind(i));
                        if isempty(idx)

                                indices = vertcat(indices, zind(i));
                                x_pred = vertcat(x_pred, LM_pred);

                                [n, ~] = size(P_pred);
                                P_pred = [ [P_pred, zeros([n, 2])];
                                                 [zeros([2, n]), W] ];

                                % Covariance Update
                                ang = angdiff(heading, -bearing);
                                Gz = [cos(ang), -r*sin(ang);
                                           sin(ang),  r*cos(ang)];
                                Gx = [eye(2), [-r*sin(ang); r*cos(ang)]];
                                
                                if n > 3
                                        Yz_upper = horzcat(eye(n), zeros([n, 2]));
                                        Yz_lower = horzcat(Gx, zeros([2, n-3]), Gz);
                                        Yz = vertcat(Yz_upper, Yz_lower);
                                else
                                        Yz = [eye(3), zeros([3,2]); [Gx, Gz]];
                                end

                                P_pred = Yz*P_pred*Yz';


                        %% Existing Landmark
                        else
                                % Expected measurement
                                LM_pred = x_pred(2*idx+2 : 2*idx+3);
                                [xdiff, ydiff] = LM_diff(vehicle_pred, LM_pred);
                                range_exp = norm([xdiff, ydiff]);
                                bearing_exp = angdiff(atan2(ydiff, xdiff), vehicle_pred(3));


                                % Jacobian of (r, β) with respect to landmark position (xi , yi)
                                Hp = [xdiff/range_exp,     ydiff/range_exp;
                                         -ydiff/range_exp^2, xdiff/range_exp^2];
                
                                Hx = zeros([2, M]);
                                Hx(:, 2*idx-1+3:2*idx+3) = Hp;
                                Hx(:,1:3) = [-xdiff/range_exp,     -ydiff/range_exp,      0;
                                                    ydiff/range_exp^2, -xdiff/range_exp^2, -1];

                                % innovation
                                nu = [measurement(1) - range_exp;
                                        angdiff(measurement(2), bearing_exp)];

                                % kalman gain
                                Hw = eye(2);
                                S = Hx*P_pred*Hx' + Hw*W*Hw';
                                K = P_pred * Hx' / S;

                                x_pred = x_pred + K*nu;
                                P_pred = P_pred - K*Hx*P_pred;
                        end

                        % plotting landmark measurement
                        plot(LM_pred(1), LM_pred(2), 'r.')

                end   
                x_est(end+1) = {x_pred};
                P_est(end+1) = {P_pred};

                plot(x_pred(1), x_pred(2), 'k.')
                pause(0.001)

        end % loop through odometry

end


%% Vehicle location
function vehicle_pred = vehicle_pos(current_pos, odo)

        x = current_pos(1);
        y = current_pos(2);
        heading = current_pos(3);

        dx = odo(1);
        dtheta = odo(2);

        vehicle_pred = [x + dx*cos(heading);
                                 y + dx*sin(heading);
                                 angdiff(heading, -dtheta)];

end


%% Landmark prediction
function LM_pred = LM_pos(vehicle_pred, measurement)

        % vehicle location
        x = vehicle_pred(1);
        y = vehicle_pred(2);
        heading = vehicle_pred(3);

        % landmark measurement
        range = measurement(1);
        bearing = measurement(2);

        % landmark location
        LM_pred = [x + range * cos(angdiff(heading, -bearing));
                           y + range *  sin(angdiff(heading, -bearing))];

end


%% x and y difference between vehicle and landmark
function [xdiff, ydiff] = LM_diff(vehicle_pos, LM_pos)

        xdiff = LM_pos(1) - vehicle_pos(1);
        ydiff = LM_pos(2) - vehicle_pos(2);

end
