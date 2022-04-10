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
        
        close all;
        
        if nargin < 1
        error('Error: Please enter a question number as a parameter');
        end
        
        % ========== Question M0 ==========
        if questionNum == 0

        end
        

        % ========== Question M1 ==========
        if questionNum == 1

        end
        

        % ========== Question M2 ==========
        if questionNum == 2

                % Load Bunny
                file = 'ex5_data/bunny.mat';
                data = load(file);
                data = struct2cell(data);
                model = data{:};
        
                angle = 5;
                distance = 0.5;
                variance = 0.000;
                nonoverlap = 1;
                disp('Find proper transform...')
        
                formatSpec = 'Maximum starting rotation is %3.0f degrees.\n';
                fprintf(formatSpec, angle)
                formatSpec = 'Maximum starting translation is %3.2f meters.\n';
                fprintf(formatSpec, distance)
                formatSpec = 'Maximum added noise is %3.3f m^2.\n';
                fprintf(formatSpec, variance)
                formatSpec = 'Models are missing %3d points each.\n';
                fprintf(formatSpec, nonoverlap)
        
                T = V2(model, angle, distance, variance, nonoverlap);
                disp('The final tranformation is...')
                disp(T)
        
        end
        

        % ========== Question M3 ==========
        if questionNum == 3
                file = 'ex5_data/ptcloud.mat';
                [xyz, ~] = loadScan(file);
                V3(xyz);
        end
        

        % ========== Question M4 ==========
        if questionNum == 4

        end


end
