% input: f -> a 9-joint robot encoded as a SerialLink class
%        qInit -> 1x9 vector denoting current joint configuration
%        posGoal -> 3x1 vector describing goal position
%        epsilon -> scalar denoting how close end effector must be before
%                   loop terminates
%        velocity -> scalar denoting desired velocity of end effector
% output: traj -> nx9 matrix that denotes arm trajectory. Each row denotes
%                 a joint configuration. The first row is the first
%                 configuration. The last is the last configuration in the
%                 trajectory.

function traj = Q3(f, qInit, posGoal, epsilon, velocity)

        % Find position and velocity
        q = qInit;
        FK = f.fkine(q);
        pos = FK.t;

        % Step through until very close to final position
        traj = [];
        step = 0.5;
        while norm(posGoal - pos) > epsilon

                % Straight line to goal in 3D space
                path = posGoal-pos;
        
                % Change in distance allowed per step
                dx = velocity * path / norm(path);

                % Intermediate goal position
                posStep = pos + dx;

                % Step through until velocity is met
                while norm(posStep-pos)>= 0.001
                        J = f.jacob0(q, 'trans');
                        dq = step * pinv(J) * dx;
                        q = q + dq';
        
                        FK = f.fkine(q);
                        pos = FK.t;
                        dx = posStep - pos;
                end

                % Add joint angles to trajectory
                traj(end+1,:) = q;          
        end
end