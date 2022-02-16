% Input: cspace -> NxN matrix: cspace(i,j)
%                  == 1 if [q_grid(i); q_grid(j)] is in collision,
%                  == 0 otherwise
% Output: padded_cspace -> NxN matrix: padded_cspace(i,j)
%                          == 1 if cspace(i,j) == 1, or some neighbor of
%                                  cell (i,j) has value 1 in cspace
%                                  (including diagonal neighbors)
%                          == 0 otherwise

function padded_cspace = C7(cspace)
        % Find lcoations of obstacles
        % Those locations with not all obstacles
        % set those locations to all obstacles
        [r,c] = find(cspace==1);
        for i = 1:length(r)
                neighbors = [r(i)-1:r(i)+1; c(i)-1:c(i)+1];
                neighbors(neighbors==0) = length(cspace);
                neighbors(neighbors>length(cspace)) = 1;
                cspace(neighbors(1,:), neighbors(2,:)) = 1;
        end
        padded_cspace = cspace;
end