% Input: q_min -> 1x4 vector of minimum angle for each joint
%        q_max -> 1x4 vector of maximum angle for each joint
%        num_samples -> Integer denoting number of samples to sample
% Output: qs -> num_samples x 4 matrix of joint angles,
%               all within joint limits

function qs = M1(q_min, q_max, num_samples)
        
        % Random 0 to 1 matrix with proper size
        qs = rand(num_samples,length(q_min));

        % Scale to proper min and max q
        qs = qs.*(q_max-q_min) + q_min;
end