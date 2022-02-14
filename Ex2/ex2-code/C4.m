% Input: distances -> NxN matrix containing the distance transform from
%                      the goal configuration
%                      == 0 if cell is unreachable
%                      == 1 if cell is an obstacle
%                      == 2 if cell is the goal
%                      >  2 otherwise
%        q_grid -> 1xN vector of angles between 0 and 2*pi, discretizing
%                  each dimension of configuration space
%        q_start -> 2x1 vector denoting the start configuration
% Output: path -> Mx2 matrix containing a collision-free path from q_start
%                 to q_goal (as computed in C3, embedded in distances).
%                 The entries of path should be grid cell indices, i.e.,
%                 integers between 1 and N. The first row should be the
%                 grid cell containing q_start, the final row should be
%                 the grid cell containing q_goal.

function path = C4(distances, q_grid, q_start)
        % Fing goal location
        [goal_r, goal_c] = find(distances == 2);
        loc_goal = [goal_r, goal_c];

        % Find start location
        [~, start_r] = min(abs(q_grid-q_start(1)));
        [~, start_c] = min(abs(q_grid-q_start(2)));
        loc_start = [start_r, start_c];

        % Set all obstacles to max value + 1
        distances(distances==1) = max(max(distances))+1;

        % First step is start location
        path = loc_start;
        while path(end) ~= loc_goal
                % Find neighbors
                neighbors = [path(end,1)-1:path(end,1)+1; path(end,2)-1:path(end,2)+1];

                %NEED TO CHECK FOR 0 and length(q_grid)
                
                square = distances(neighbors(1,:),neighbors(2,:));
        
                % Check the minimum value
                [r,c] = find(square == min(min(square)));
                % Break ties
                if length(r)>1
                        idx = randi([1,length(r)],1);
                        r = r(idx);
                        c = c(idx);
                end
        
                % Add location to path
                next = [path(end,1)+r-2, path(end,2)+c-2];
                path = cat(1,path,next);
        end
end