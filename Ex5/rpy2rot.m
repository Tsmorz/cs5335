function [R] = rpy2rot(rpy)
%RPY2ROT Summary of this function goes here
%   Detailed explanation goes here

r = rpy(1);
p = rpy(2);
y = rpy(3);

X = eye(3);
X(2:3, 2:3) = [cosd(r), -sind(r); sind(r), cosd(r)];

Y = eye(3);
Y(1:2:3, 1:2:3) = [cosd(p), sind(p); -sind(p), cosd(p)];

Z = eye(3);
Z(1:2, 1:2) = [cosd(y), -sind(y); sind(y), cosd(y)];

R = Z*Y*X;

end

