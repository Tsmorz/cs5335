% input: f -> a 9-joint robot encoded as a SerialLink class
%        qInit -> 1x9 vector denoting current joint configuration
%        posGoal -> 3x1 vector denoting the target position to move to
% output: q -> 1x9 vector of joint angles that cause the end
%              effector position to reach <position>
%              (orientation is to be ignored)

function q = Q2i(f, qInit, posGoal)
    
        % Only need first 3dof for posGoal
        q = qInit;
        q(7:9) = 0;
        
        % Set q1 so first angle aligns with goal x-y coordinate
        q(1) = atan(posGoal(2)/posGoal(1));
        
        % Adjust q2 and q4 for posGoal
        l1 = f.d(3);
        l2 = f.d(5)+f.a(8)+f.a(9);
        q(4) = pi - acos( (l1^2+l2^2-norm(posGoal)^2) / (2*l1*l2) );
        
        gamma = asin(l2*sin(q(4)) / norm(posGoal));
        alpha = asin( posGoal(3) / norm(posGoal) );
        q(2) = gamma + alpha - pi/2;
        
        FK =  f.fkine(q);
        diff = FK.t - posGoal;
        disp(diff)

end