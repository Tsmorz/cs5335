% input: f -> a 9-joint robot encoded as a SerialLink class
%        qInit -> 1x9 vector denoting current joint configuration
%        posGoal -> 3x1 vector denoting the target position to move to
% output: q -> 1x9 vector of joint angles that cause the end
%              effector position to reach <position>
%              (orientation is to be ignored)

function q = Q2(f, qInit, posGoal)

        % Random initial joint angles
        q = pi/100 * randi([-100, 100], 1, length(qInit));

        % Set q1 so first angle aligns with goal x-y coordinate
        q(1) = atan(posGoal(2)/posGoal(1));

        % at position if hand is closed
        q(7:9) = 0;

        % repeat k times
        i = 0;
        step = 2*pi / 50;
        while i <= 50
                % find current position
                FK = f.fkine(q);
                pos = FK.t;

                dx = posGoal - pos;
                J = f.jacob0(q,'trans');
                dq = step * pinv(J) * dx;
                q = q + dq';

                i = i+1;

        end

        FK = f.fkine(q);
        pos = FK.t;
        norm(posGoal-pos)

end