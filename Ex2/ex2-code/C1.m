% Input: robot -> A 2-DOF robot encapsulated in a MATLAB cell with fields:
%                 robot.link1, robot.link2, robot.pivot1, robot.pivot2
%                 See description in hw2_cspace and demo code in C1.
%        q -> 2x1 vector denoting the configuration to plot the robot at

function C1(robot, q)
    % The following code plots the robot in configuration q = [0; 0].
    % You should remove the following code and replace it with code that
    % plots the robot links and pivots at the provided input configuration.
    
    % Translate frame origins
    origin1_at0 = robot.pivot1;
    origin2_at0 = origin1_at0 + robot.pivot2;
    % Compute link polygon corners
    link1_at0 = robot.link1 + origin1_at0;
    link2_at0 = robot.link2 + origin2_at0;
    % Plot the links
    plot(polyshape(link1_at0(1,:), link1_at0(2,:)), 'FaceColor', 'r');
    plot(polyshape(link2_at0(1,:), link2_at0(2,:)), 'FaceColor', 'b');
    % Plot the pivot points
    plot(origin1_at0(1), origin1_at0(2), 'k.', 'MarkerSize', 10);
    plot(origin2_at0(1), origin2_at0(2), 'k.', 'MarkerSize', 10);

end