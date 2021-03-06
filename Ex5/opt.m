% Tony Smoragiewicz
% Ex5

function opta()

        figure(1)
        hold on
        sgtitle('Robot Position')

        % original cost function
        x = opt(@cost1);
        subplot(2,2,1)
        hold on
        title('a')
        helpPlot(x)
        
        % new cost 1
        x = opt(@cost2);
        subplot(2,2,2)
        hold on
        title('b1')
        helpPlot(x)

        % new cost 2
        x = opt(@cost3);
        subplot(2,2,3)
        hold on
        title('b2')
        helpPlot(x)

        % new cost 3
        x = opt(@cost4);
        subplot(2,2,4)
        hold on
        title('b3')
        helpPlot(x)

end
%% Optimization
function x = opt(fun)
        xstart = [2; 2];
        xgoal = [4; 0];

        T = 20;

        x0 = zeros([2*(T-1), 1]);

        A = [];
        b = [];

        Aeq = repmat(eye(2), [1, T-1]);
        beq = xgoal - xstart;

        u = fmincon(fun, x0, A, b, Aeq, beq);

        u = reshape(u, [2, T-1]);
        x = horzcat(xstart, cumsum(u, 2) + xstart);
end


%% cost functions
% sum of all squared commands
function f = cost1(u) 
         f = dot(u, u); 
end

% cost to go is smaller at the start for both x and y
function f = cost2(u)
        steps = linspace(0.1, 0.5, length(u)/2);
        steps = repelem(steps, 2);
        x = u.*steps';
        f = dot(x,x);  
end

% cost to go is lower at the start for y
function f = cost3(u)
        steps = linspace(0.1, 0.9, length(u)/2);
        steps = repelem(steps, 2);
        steps(1:2:end) = 1;
        x = u.*steps';
        f = dot(x,x);  
end

% cost to go is lower at the start for x
function f = cost4(u)
        steps = linspace(0.1, 0.2, length(u)/2);
        steps = repelem(steps, 2);
        steps(2:2:end) = 1;
        x = u.*steps';
        f = dot(x,x);  
end