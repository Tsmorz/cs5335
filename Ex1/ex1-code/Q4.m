% input: f -> a 9-joint robot encoded as a SerialLink class
%        qInit -> 1x9 vector denoting current joint configuration
%        circle -> 3xn matrix of Cartesian positions that describe the
%                  circle
%        velocity -> scalar denoting desired velocity of end effector
% output: traj -> nx9 matrix that denotes arm trajectory. Each row denotes
%                 a joint configuration. The first row is the first
%                 configuration. The last is the last configuration in the
%                 trajectory. The end effector should trace a circle in the
%                 workspace, as specified by the input argument circle.
%                 (orientation is to be ignored)

function traj = Q4(f, qInit, circle, velocity)
        % Set tolerance (same as Q3)
        epsilon = 0.05;
        traj = [];
        
        % Loop through all points in desired path
        [~, c] = size(circle);
        for i = 1:c-1
                posGoal = circle(:,i+1);
                
                % Find trajectory that connects the points
                q = Q3(f, qInit, posGoal, epsilon, velocity);

                % Update current angles
                qInit = q(end,:);

                % Append it to previous set of joints
                traj = cat(1,traj,q);
        end
end