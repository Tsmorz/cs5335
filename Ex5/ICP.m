function [t_final] = ICP(X, P)
%ICP Summary of this function goes here
%   Detailed explanation goes here

        t_final = eye(4);
        T = eye(4);
        E = zeros([1,200]);
        
        for i = 1:200
                clf;
                Y = P;
                [model, new] = closestCloud(X, Y);
                
                W = model' * new;
                [U, S, V] = svd(W);
                R = U*V';
                t = mean(model,1)' - R*mean(new,1)';

                % track final transformation
                T(1:3, 1:3) = R;
                T(1:3,4) = t;
                t_final = T*t_final;
        
                % transform input point cloud
                Yt = R*new' + t;
                P = R*P + t;

                % error calculations
                eigen = sum(sum(S));
                [N, ~] = size(model);
                E(i) = 1/N * sum( vecnorm(model' - Yt).^2 );

                % plot point cloud
                subplot(2,1,1)
                axis('equal')
                view(0, 90)
                hold on
                plotData3(X, 'b.')
                plotData3(P, 'ro')

                % plot error
                subplot(2,1,2)
                semilogy(E(1:i))
                ylim([0.00001, 0.01])
                xlabel('ICP Iterations')
                ylabel('Error Value')
                pause(0.001)
        
                
                if E(i) < 0.00001
                        break
                end
        
        end

end