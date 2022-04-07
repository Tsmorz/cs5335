% Tony Smoragiewicz
% V2 - ICP algorithm

close all

file = 'ex5_data/bunny.mat';
M = loadScan(file);

figure(1)
rpy = 180*rand([1,3]);
rot = SE3.rpy(rpy);
tran = rand([3,1]);
test = rot * M + tran;
[R, t] = ICP(M, test);

figure(2)
test = rot * (M + 0.01*rand(size(M))) + tran;
[R, t] = ICP(M, test);
 
figure(3)
[r, c] = size(M);
[R, t] = ICP( M(:,1:c-20), M(:,21:c));

ptCloud = M;
point = M(:,1)'
K = 1;
[indices,dists] = findNearestNeighbors(ptCloud,point,K);

% Iterative Closest Point alg. for point clouds
function [R, t] = ICP(X, P)

        % subtract mean from point cloud
        mu_x = mean(X, 2);
        Xt = X - mu_x;

        Y = P;
        for i = 1:5
                mu_y = mean(Y, 2);
                Yt = Y - mu_y;
        
                W = Xt * Yt';
        
                % SVD on mean centered point cloud
                [U, S, V] = svd(W);
                eigen = [S(1,1), S(2,2), S(3,3)];

                R = U*V';
                t = mu_x - R*mu_y;
                Y = R*Y + t;
                imatch(Xt, Y)

                [~, N] = size(X);
                E = 1/N * sum( norm(Xt - Y).^2 )
                minE = sum(norm(Xt)^2 + norm(Yt)^2) - 2*(sum(eigen))


        end

        subplot(1,2,1)
        axis('equal')
        view(0, 90)
        hold on
        plotData(X, 'b.')
        plotData(P, 'ro')
        
        subplot(1,2,2)
        axis('equal')
        view(0, 90)
        hold on
        plotData(X, 'b.')
        plotData(Y, 'ro')

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