% Helper function to plot End Effector position
% Inputs are robot and joint angles 
function plot_path(qs, robot, marker)

        % Plot EF position for smoothed path
        X = zeros([length(qs),3]);
        [r,~] = size(qs);
        for i = 1:r
                X(i,:) = robot.fkine(qs(i,:)).t;
        end
        plot3(X(:,1), X(:,2), X(:,3), marker)

end