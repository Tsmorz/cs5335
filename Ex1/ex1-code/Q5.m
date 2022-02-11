% input: f1 -> an 9-joint robot encoded as a SerialLink class for one
%              finger
%        f2 -> an 9-joint robot encoded as a SerialLink class for one
%              finger
%        qInit -> 1x11 vector denoting current joint configuration.
%                 First seven joints are the arm joints. Joints 8,9 are
%                 finger joints for f1. Joints 10,11 are finger joints
%                 for f2.
%        f1Target, f2Target -> 3x1 vectors denoting the target positions
%                              each of the two fingers.
% output: q -> 1x11 vector of joint angles that cause the fingers to
%              reach the desired positions simultaneously.
%              (orientation is to be ignored)

function q = Q5(f1, f2, qInit, f1Target, f2Target)

        % IK up to wrist
        function q = IK_wrist(f, qInit, posGoal)
        
                % Initial joint angles gives a better path
                q = qInit;
        
                % find current position
                FK = f.fkine(q);
                dx = posGoal - FK.t;
        
                % Iterate until a solution is 1mm from the goal
                step = 1.25;
                while norm(dx) > 0.01
                        J = f.jacob0(q,'trans');
                        dq = step * pinv(J) * dx;
                        dq(1) = 0;
                        dq(8:9) = 0;
                        q = q + dq';
                        FK = f.fkine(q);
                        pos = FK.t;
                        dx = posGoal - pos;
                end
                
        end
        
        % IK for the f1 finger and part of wrist
        function q = IK_grip1(f, qInit, posGoal)
        
                % Initial joint angles gives a better path
                q = qInit;
        
                % find current position
                FK = f.fkine(q);
                dx = posGoal - FK.t;
        
                % Iterate until a solution is 1mm from the goal
                step = 5;
                while norm(dx) > 0.001
                        J = f.jacob0(q,'trans');
                        dq = step * pinv(J) * dx;
                        dq(1:5) = 0;
                        q = q + dq';
                        FK = f.fkine(q);
                        pos = FK.t;
                        dx = posGoal - pos;
                end
                
        end

        % IK for f2 only
        function q = IK_grip2(f, qInit, posGoal)
        
                % Initial joint angles gives a better path
                q = qInit;
        
                % find current position
                FK = f.fkine(q);
                dx = posGoal - FK.t;
        
                % Iterate until a solution is 1mm from the goal
                step = 5;
                while norm(dx) > 0.001
                        J = f.jacob0(q,'trans');
                        dq = step * pinv(J) * dx;
                        dq(1:7) = 0;
                        q = q + dq';
                        FK = f.fkine(q);
                        pos = FK.t;
                        dx = posGoal - pos;
                end
                
        end

        % Want wrist to come close to center of ball
        q = qInit;
        q(8:11) = 0;
        posGoal = mean([f1Target,f2Target],2);
        tip = f1.a(9);
        posGoal = posGoal + 0.5 * tip * posGoal / norm(posGoal);

        % Find q1 angle to center of ball
        q(1) = atan(posGoal(2)/posGoal(1));
       
        % Find angles up to wrist while keeping q1 constant
        qWrist = IK_wrist(f1, q(1:9), posGoal);
        q(1:7) = qWrist(1:7);
        
        % Find f1 finger angles while keeping wrist constant
        q(8:11) = qInit(8:11);
        qGrip = IK_grip1(f1, q(1:9), f1Target);
        q(6:9) = qGrip(6:9);

        % Find f2 finger angles while keeping wrist constant
        qGrip = IK_grip2(f1, [q(1:7), qInit(10:11)], f2Target);
        q(10:11) = qGrip(8:9);

end