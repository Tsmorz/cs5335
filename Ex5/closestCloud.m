function [model, input] = closestCloud(original1, original2)
%CLOSESTCLOUD Summary of this function goes here
%   Detailed explanation goes here

% check data set sizes for comparison
original1 = original1';
original2 = original2';
[r1, c1] = size(original1);
[r2, ~] = size(original2);

if r1 < c1
        original1 = original1';
        original2 = original2';
        [r1, ~] = size(original1);
        [r2, ~] = size(original2);
end

% different number of points
if r1 ~= r2
        if r1 > r2
                rows = randperm(r1, r2);
                model = original1(rows,:);
                input = original2;

        else % r1 < r2
                rows = randperm(r2, r1);
                input = original2(rows,:);
                model = original1;
        end
else
        model = original1;
        input = original2;
end

% loop through smallest number of rows
r = min([r1, r2]);
idx = zeros([r, 2]);
for i = 1:r
        point = model(i,:);
        diffsq = (input - point).^2;
        distance = sqrt(sum(diffsq, 2));
        loc = find(distance == min(distance));
        idx(i,:) = [i, loc];
end

input = input(idx(:,2),:);

end