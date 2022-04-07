% Input: x0 -> 3x1 vector denoting the initial vehicle state
%        T -> Scalar denoting the number of time steps in the trajectory
% Output: x_truth -> 3xT matrix containing true (non-noisy) trajectory
%         odo_truth -> 2xT matrix containing true (non-noisy)
%                      odometry readings for T time steps

function [x_truth, odo_truth] = E6(x0, T, map)

        landmarks = map.map(:);
        landmarks = reshape(landmarks, [length(landmarks)/2, 2]);

        x_truth = zeros([3,T]);
        x_truth(:,1) = x0;

        odo_truth = zeros([2,T]);
        odo_truth(1,:) = 0.1;

        figure(1)
        plot(landmarks(:,1), landmarks(:,2), 'k*')
        hold on
        axis('equal')

        % loop through all time steps
        for i = 1:T-1
                x = x_truth(1, i);
                y = x_truth(2, i);
                heading = x_truth(3, i);

                % Plotting vehicle location
                plot(x, y, 'k.')
                pause(0.01)

                % position update
                dx = odo_truth(1,i);
                k = waitforbuttonpress;
                value = double(get(gcf,'CurrentCharacter'));
                disp('Make sure the plot is active and use your arrow keys')
                if value == 28
                        odo_truth(2,i) = 0.0545;
                elseif value == 29
                        odo_truth(2,i) = -0.0545;
                else
                        odo_truth(2,i) = 0;
                end
                dtheta = odo_truth(2,i);

                x_truth(:, i+1) = [x + dx*cos(heading);
                                                        y + dx*sin(heading);
                                                        angdiff(heading, -dtheta)];

        end
    
end
