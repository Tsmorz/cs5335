% Input: robot -> A 2-DOF robot encapsulated in a MATLAB cell with fields:
%                 robot.link1, robot.link2, robot.pivot1, robot.pivot2
%                 See description in hw2_cspace and demo code in C1.
%        q -> 2x1 vector denoting the configuration to plot the robot at

function C1(robot, q)
        % Find the new coordinates post transformation
        [link1, link2, origin1, origin2] = q2poly(robot,q);

        % Plot the links
        plot(polyshape(link1(1,:), link1(2,:)), 'FaceColor', 'r');
        plot(polyshape(link2(1,:), link2(2,:)), 'FaceColor', 'b');
        
        % Plot the pivot points
        plot(origin1(1), origin1(2), 'k.', 'MarkerSize', 10);
        plot(origin2(1), origin2(2), 'k.', 'MarkerSize', 10);

end