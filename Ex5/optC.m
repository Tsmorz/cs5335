% Tony Smoragiewicz
% Ex5


function x = optC(fun)
        xstart = [2; 2; 5];
        xgoal = [4; 0; 0.01];

        T = 20;

        x0 = zeros([3*(T-1), 1]);

        A = [];
        b = [];

        Aeq = repmat(eye(2), [1, T-1]);
        beq = xgoal - xstart;

        u = fmincon(fun, x0, A, b, Aeq, beq);

        u = reshape(u, [2, T-1]);
        x = horzcat(xstart, cumsum(u, 2) + xstart);
end
