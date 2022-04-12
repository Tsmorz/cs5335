%% Surface Normal from point
function [normal, ratio, points] = eigenNorm(point, radius, xyz)

        dist = sqrt( sum( (xyz-point).^2, 2 ) );
        points = xyz(dist<radius, :);

        mu = mean(points, 1, 'omitnan');
        pdiff = points - mu;

        Sigma = pdiff' * pdiff;
        [vec, lambda] =eig(Sigma);

        lambda = [lambda(1,1), lambda(2,2), lambda(3,3)];
        idx = find(lambda == min(lambda));
        normal = vec(:, idx)';

        ratio = min(lambda) / max(lambda);
        
end

