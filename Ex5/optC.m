% Tony Smoragiewicz
% Ex5 - OPTc

% light-dark domain planning
function x = optC(T)

        xstart = [2; 2; 5];
        xgoal = [4; 0; 0.01];

        x0 = zeros([2*(T-1), 1]);

        A = [];
        b = [];

        Aeq = repmat(eye(2), [1, T-1]);
        beq = xgoal(1:2) - xstart(1:2);

        lb = [];
        ub = [];

        nlcon = @nonlincon;
        u = fmincon(@costX, x0, A, b, Aeq, beq, ub, lb, nlcon);
        u = reshape(u, [2, T-1]);

        
        plotRobot(u, xstart, T);
end



%% cost functions
% sum of all squared commands
function f = cost(u)
         f = dot(u, u);
end

% cost to go is lower at the start for y
function f = costY(u)
        steps = linspace(0.5, 1, length(u)/2);
        steps = repelem(steps, 2);
        steps(1:2:end) = 1;
        x = u.*steps';
        f = dot(x,x);  
end

% cost to go is lower at the start for x
function f = costX(u)
        steps = linspace(1, 0, length(u)/2);
        steps = repelem(steps, 2);
        steps(2:2:end) = 1;
        x = u.*steps';
        f = dot(x,x);  
end



%% nonlinear constraint
function [c, ceq] = nonlincon(u)

        T = length(u)/2+1;
        xstart = [2; 2; 5];
        
        belief = zeros([3,T]);
        u = u';
        u = reshape(u, [2, T-1]);
        belief(:, 1) = xstart;
        for i = 1:T-1
                belief(1:2, i+1) = belief(1:2,i) + u(:,i);
                x = belief(1, i+1);
                y = belief(2, i+1);
                var = belief(3, i);
                var = kalman([x, y, var]);
                belief(3, i+1) = var(1,1);
        end

        c = belief(3, end) - 0.01;
        ceq = 0;

end



%% robot uncertainty
function var = kalman(belief)
        % prediction
        transition = 0.01*eye(2);
        var_pred = belief(3)*eye(2) + transition;

        % innovation
        sig_w = 1/2 * (5-belief(1))^2 + 0.01;
        K = var_pred / (var_pred + sig_w*eye(2));

        % update
        var = (eye(2) - K) * var_pred;
end



%% plot circle
function circle(x,y,r)
        ang=0:0.01:2*pi; 
        xp=r*cos(ang);
        yp=r*sin(ang);
        plot(x+xp,y+yp, 'r');
end



%% final path plot
function plotRobot(u, xstart, T)
        belief = zeros([3,T]);
        belief(:, 1) = xstart;
        for i = 1:T-1
                belief(1:2, i+1) = belief(1:2,i) + u(:,i);
                x = belief(1, i+1);
                y = belief(2, i+1);
                var = belief(3, i);
                var = kalman([x, y, var]);
                belief(3, i+1) = var(1,1);
        end


        % plot robot position
        hold on
        x = belief(1,:);
        y = belief(2,:);
        plot(x, y, 'k-')

        % plot light-dark
        x = min(x)-2:0.5:max(x)+2;
        y = min(y)-2:0.5:max(y)+2;
        [X, Y] = meshgrid(x,y);
        Z = -1/2 * (5 - X).^2;
        surf(X, Y, Z);

        % uncertainty
        for i = 1:T
                x = belief(1,i);
                y = belief(2,i);
                r = belief(3,i)/3;
                circle(x,y,r)
        end

        shading interp
        colormap gray
        title('Robot Position')
        xlabel('x position')
        ylabel('y position')
        view(2);

end