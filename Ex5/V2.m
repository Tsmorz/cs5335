%% Tony Smoragiewicz
% V2 - ICP algorithm

file = 'ex5_data/bunny.mat';
M = loadScan(file);

close all

[r, c] = size(M);

limit = 100;
M1 = M(:,1:end-limit);

mismatch = M(:,limit:end);

noise = 0.0001*rand(size(mismatch));

tran = rand([3,1])/10;

rpy = 90*rand([1,3]);
rot = SE3.rpy(rpy);

M2 = rot*mismatch+tran + noise;
[R, t] = ICP(M1, M2);




% Find closest point
function [model, input] = closestCloud(original1, original2)

        % check data set sizes for comparison
        original1 = original1';
        original2 = original2';
        [r1, c1] = size(original1);
        [r2, c2] = size(original2);

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

%         figure(1)
%         clf
%         axis('equal')
%         view(0, 90)
%         hold on
%         plotData(model', 'b.')
%         plotData(input', 'ro')

        input = input(idx(:,2),:);

end


% Iterative Closest Point alg. for point clouds
function [R, t] = ICP(X, P)

        if size(X) == size(P)
                %[model, new] = closestCloud(X, P);
                Xt = X - mean(X,2);
                Yt = P - mean(P,2);
                W = Xt * Yt';
                [U, S, V] = svd(W);
                %eigen = [S(1,1), S(2,2), S(3,3)];
                R = U*V';
                t = mean(X,2) - R*mean(P,2);
        else
                for i = 1:200
                        clf;
                        Y = P;
                        [model, new] = closestCloud(X, Y);
                        
                        W = model' * new;
                
                        % SVD on mean centered point cloud
                        [U, S, V] = svd(W);
        
                        R = U*V';
                        t = mean(model,1)' - R*mean(new,1)';
                        Yt = R*new' + t;

                        [N, ~] = size(model);
                        E = 1/N * sum( vecnorm(model' - Yt).^2 );
                        P = R*P + t;


                        hold off
                        axis('equal')
                        view(0, 90)
                        hold on
                        plotData(X, 'b.')
                        plotData(P, 'ro')
                        pause(0.001)
                end
     

        end

end

%% Load model
function model = loadScan(file)
        % Load Bunny
        data = load(file);
        
        % Convert to array
        data = struct2cell(data);
        model = data{:};
end

%% Plot 3D points
function plotData(data, marker)
        x = data(1,:);
        y = data(2,:);
        z = data(3,:);
        scatter3(x, y, z, marker)
end