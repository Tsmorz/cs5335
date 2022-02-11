% input: f -> a 9-joint robot encoded as a SerialLink class
%        qInit -> 1x9 vector denoting current joint configuration
%        posGoal -> 3x1 vector denoting the target position to move to
% output: q -> 1x9 vector of joint angles that cause the end
%              effector position to reach <position>
%              (orientation is to be ignored)

function q = Q2(f, qInit, posGoal)

        % Initial joint angles gives a better path
        q = qInit;

        % find current position
        FK = f.fkine(q);
        dx = posGoal - FK.t;

        % Iterate until a solution is 1mm from the goal
        step = 0.5;
        while norm(dx) > 0.001
                J = f.jacob0(q,'trans');
                dq = step * pinv(J) * dx;
                q = q + dq';
                FK = f.fkine(q);
                pos = FK.t;
                dx = posGoal - pos;
        end
        
end