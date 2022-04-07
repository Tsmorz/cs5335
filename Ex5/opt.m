% Tony Smoragiewicz
% Ex5

function opt()

        T = 20;
        x0 = [2; 2];
        x_goal = [4; 0];

        x = zeros([2, T])
        u = zeros([2, T-1])

        x(:,1) = x0
        x(:, end) = x_goal

        A = []
        b = []

        fun = @(x) x(2)
        x = fmincon(@cost, x0, A, b)

end

% cost function to minimize
function f = cost(u)

        R = eye(2);
        f = u'*R*u;

end