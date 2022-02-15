% Input: cspace -> NxN matrix: cspace(i,j)
%                   == 1 if [q_grid(i); q_grid(j)] is in collision,
%                   == 0 otherwise
%        q_grid -> 1xN vector of angles between 0 and 2*pi, discretizing
%                  each dimension of configuration space
%        q_goal -> 2x1 vector denoting the goal configuration
% Output: distances -> NxN matrix containing the distance transform from
%                      the goal configuration
%                      == 0 if cell is unreachable
%                      == 1 if cell is an obstacle
%                      == 2 if cell is the goal
%                      >  2 otherwise

function distances = C3(cspace, q_grid, q_goal)
        distances = cspace;

        % Goal State = 2
        [~, r] = min(abs(q_grid-q_goal(1)));
        [~, c] = min(abs(q_grid-q_goal(2)));
        distances(r, c) = 2;
        
        % Wavefront Planner
        i = 0;
        while min(min(cspace)) < 1
                [rx,cx] = find(distances == (2+i));
                wall = [rx,cx];
                done = true(1);

                % Loop through all of front line
                for j = 1:length(rx)
                        idx = wall(j,:);
                        
                        % Shifted rows
                        for k = -1:1
                                % Shifted columns
                                for l = -1:1

                                        %%%%%%%%%%%%%%%%%%%%
                                        % Catch for values >2pi or <1
                                        r = idx(1)+k;
                                        if r > length(distances)
                                                r = 1;
                                        elseif r == 0
                                                r = length(distances);
                                        elseif r<0
                                                disp('negative')
                                        end

                                        % Catch for values >2pi or <1
                                        c = idx(2)+l;
                                        if c > length(distances)
                                                c = 1;
                                        elseif c == 0
                                                c = length(distances);
                                        elseif c<0
                                                disp('negative')
                                        end
                                        %%%%%%%%%%%%%%%%%%%%%

                                        if distances(r, c) == 0
                                                distances(r, c) = 3+i;
                                                % Change done to false
                                                done = false(1);
                                        end
                                end
                        end
                end                

                % Exit while loop if wave didn't spread
                if done
                        break
                end
                i = i+1;
                %imshow(flipud(distances'), [min(min(distances)), max(max(distances))]);
        end
end