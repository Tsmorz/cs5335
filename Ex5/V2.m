%% Tony Smoragiewicz
function [T] = V2(M, angle, distance, variance, overlap)
% V2 - ICP algorithm
%   Detailed explanation goes here

        M1 = M(:,1:end-overlap);
        mismatch = M(:, overlap:end);

        noise = variance*rand(size(mismatch));

        tran = distance*rand([3,1]);
        
        rpy = angle*rand([1,3]);
        rot = SE3.rpy(rpy);
        
        M2 = rot*mismatch+tran + noise;
        [T] = ICP(M1, M2);

end