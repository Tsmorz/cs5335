% Input: robot -> A 2-DOF robot encapsulated in a MATLAB cell with fields:
%                 robot.link1, robot.link2, robot.pivot1, robot.pivot2
%                 See description in hw2_cspace and demo code in C1.
%        q -> 2x1 vector denoting the configuration to convert to polygons
% Output: poly1 -> A polyshape object denoting the 2-D polygon describing
%                  link 1
%         poly2 -> A polyshape object denoting the 2-D polygon describing
%                  link 2
%         pivot1 -> 2x1 vector denoting the 2-D position of the pivot point
%                   of link 1 (frame 1 origin), with respect to base frame
%         pivot2 -> 2x1 vector denoting the 2-D position of the pivot point
%                   of link 2 (frame 2 origin), with respect to base frame

function [poly1, poly2, pivot1, pivot2] = q2poly(robot, q)

        % Angle notation makes the code cleaner
        q1 = q(1);
        q12 = q(1) + q(2);
        
        % Find parameters for for link
        pivot1 = robot.pivot1;
        poly1 = [cos(q1), -sin(q1); sin(q1), cos(q1)] * robot.link1 + robot.pivot1;
        poly1 = polyshape(poly1');
        
        % Find parameters for second link
        pivot2 =  [cos(q1), -sin(q1); sin(q1), cos(q1)] * robot.pivot2 + robot.pivot1;
        poly2 = [cos(q12), -sin(q12); sin(q12), cos(q12)] * robot.link2 + pivot2;
        poly2 = polyshape(poly2');

end