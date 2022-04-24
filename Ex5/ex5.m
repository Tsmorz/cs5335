% DO NOT MODIFY THIS FILE!
% Input: questionNum -> Integer between 0 and 6 that denotes question
%                       number to run.
%        samples, adjacency -> Optional pre-computed PRM samples and
%                              adjacency matrix to avoid re-computing it in
%                              question M3
% Output: samples, adjacency -> If PRM roadmap was computed, you can save
%                               it and pass it in on later calls to
%                               hw2_motion to avoid re-computing it

function [] = ex5(questionNum)
        
        close all
        format short
        format compact
        
        if nargin < 1
        error('Error: Please enter a question number as a parameter');
        end
        
        % ========== Question M0 ==========
        if questionNum == 0
                f1 = figure(1);
                hold on
                sgtitle('Robot Position')
                movegui(f1, 'northwest')
                optAB();

                % light dark domain
                figure(2);
                optC(15);
        end

        
        % ========== Question M1 ==========
        if questionNum == 1
                V1();
                
        end
        
        % ========== Question M2 ==========
        if questionNum == 2

                % Load Bunny
                file = 'ex5_data/bunny.mat';
                data = load(file);
                data = struct2cell(data);
                M = data{:};

                % force matrix to be horizontal
                [r, c] = size(M);
                if r > c
                        M = M';
                        [r, c] = size(M);
                end
                
                % apply random transformation
                M1 = M;
                M2 = M;

                % add noise
                M2 = M2 + 0.01*rand(size(M2));

                % apply transform
                t = 1/5 * rand([3, 1]);
                rpy = 45* rand([1,3]);
                R = rpy2rot(rpy);
                M2 = R * M2 + t;

                % remove some points
                num = 0;
                M1 = M1(:, 1:end-num);
                M2 = M2(:, 1+num:end);

                disp('The initial tranformation is...')
                initial = eye(4);
                initial(1:3, 1:3) = R;
                initial(1:3, 4) = t;
                disp(initial)
                

                f = figure(1);
                movegui(f, 'northwest')

                steps = 100;
                E = zeros([steps,1]);
                Tfinal = eye(4);
                for i = 1:steps

                        [R, t, error, dist] = ICP(M1, M2);
                        M2 = R*M2 + t; 
                        E(i) = error;

                        % plot point cloud
                        clf;
                        subplot(2, 2, [1 3])
                        hold on
                        axis('equal')
                        view(0, 90)
                        plotData3(M1, 'b.')
                        plotData3(M2, 'r.')
        
                        % plot error
                        subplot(2,2,2)
                        semilogy(E(1:i))
                        ylabel('Error Value')
                        if i > 1
                                ylim([min(E(1:i)), max(E)])
                        end
                        xlabel('ICP Iterations')
        
                        % plot histogram of closest point distances
                        subplot(2,2,4)
                        histogram(dist, 30)
                        set(gca,'YScale','log')
                        xlim([0, max(dist)+0.01])
                        xlabel('Distance to closest point')

                        hold off
                        drawnow

                        T = eye(4);
                        T(1:3, 1:3) = R;
                        T(1:3, 4) = t;
                        Tfinal = T*Tfinal;

                        if E(i) < 0.000001
                                disp('Exited with a close alignment.')
                                break
                        end
                        if i == steps
                                disp('Exited with maximum number of ICP iterations.')
                        end
                end

                disp('The final tranformation is...')
                final = inv(Tfinal);
                disp(final)

        end
        
        % ========== Question M3 ==========
        if questionNum == 3
                file = 'ex5_data/ptcloud.mat';
                [xyz, rgb] = loadScan(file);
                V3(xyz, rgb);
        end
        
        % ========== Question M4 ==========
        if questionNum == 4
                pcloud();
        end

end
