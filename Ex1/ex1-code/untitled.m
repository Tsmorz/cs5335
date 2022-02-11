        % Set q1 so first angle aligns with goal x-y coordinate
        q(1) = atan(posGoal(2)/posGoal(1));

        % at position if hand is closed
        q(7:9) = 0;