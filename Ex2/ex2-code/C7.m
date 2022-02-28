% Input: cspace -> NxN matrix: cspace(i,j)
%                  == 1 if [q_grid(i); q_grid(j)] is in collision,
%                  == 0 otherwise
% Output: padded_cspace -> NxN matrix: padded_cspace(i,j)
%                          == 1 if cspace(i,j) == 1, or some neighbor of
%                                  cell (i,j) has value 1 in cspace
%                                  (including diagonal neighbors)
%                          == 0 otherwise

function padded_cspace = C7(cspace)
 
        % Find locations of obstacles
        % Those locations with not all obstacles
        % set those locations to all obstacles
        padded_cspace = cspace;
        [r,c] = find(padded_cspace==1);
        for j = 1:length(r)
                neighbors = [r(j)-1:r(j)+1; c(j)-1:c(j)+1];
                neighbors(neighbors==0) = length(padded_cspace);
                neighbors(neighbors>length(padded_cspace)) = 1;
                padded_cspace(neighbors(1,:), neighbors(2,:)) = 1;
        end

end