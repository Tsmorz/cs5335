function [R, t, error, distances] = ICP(X, P)
%ICP Summary of this function goes here
%   Detailed explanation goes here

        [model, new, distances] = closestCloud(X, P);
        
        W = model' * new;
        [U, S, V] = svd(W);

        R = U*V';
        t = mean(model,1)' - R*mean(new,1)';

        % transform input point cloud
        Y = R*new' + t;

        % error calculations
        eigen = sum(sum(S));
        [N, ~] = size(model);
        diff = model - Y';
        dist = vecnorm(diff, 1, 2);
        error = 1/N * dot(dist, dist');

end


%% Helper function
function [model, input, distances] = closestCloud(original1, original2)
%CLOSESTCLOUD Summary of this function goes here
%   Calculate closest points between two point clouds

        % check data set sizes for comparison
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
        distances = zeros([r, 1]);
        for i = 1:r
                point = model(i,:);
                dist = vecnorm(input - point, 2, 2);
                loc = find(dist == min(dist));
                idx(i,:) = [i, loc];
                distances(i) = min(dist);
        end
        
        input = input(idx(:,2),:);

end