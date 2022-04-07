% Input: V -> 2x2 matrix denoting the process noise in (forward, angular)
%        W -> 2x2 matrix denoting the sensing noise in (range, bearing)
%        x0 -> 3x1 vector denoting the initial vehicle state mean
%        P0 -> 3x3 matrix denoting the initial vehicle state covariance
%        range -> Scalar denoting the maximum range of the sensor
%        fov -> 1x2 vector denoting the [min_bearing, max_bearing]
%               (field of view) that can be detected by the sensor
% Output: ekf_l -> Robotics toolbox EKF object
%                  after localizing against a known map
%         ekf_m -> Robotics toolbox EKF object
%                  after estimating a map with known vehicle poses
%         ekf_s -> Robotics toolbox EKF object after performing SLAM

function [ekf_l, ekf_m, ekf_s] = E0(V, W, x0, P0, range, fov)

        t = 1000;

        rng(0)
        map = LandmarkMap(20);

        % Localization
        veh = Bicycle('covar', V, 'x0', x0);
        veh.add_driver( RandomPath(map.dim) );
        sensor = RangeBearingSensor(veh, map, 'covar', W, 'angle',...
                fov, 'range', range, 'animate');
        ekf_l = EKF(veh, V, P0, sensor, W, map);
        ekf_l.run(t);

        map.plot();
        veh.plot_xy();
        ekf_l.plot_xy('r');
        ekf_l.plot_ellipse('k');

        visualize({}, {}, [], map, veh, 'n', []);
        pause(1)


        % Mapping
        veh = Bicycle();
        veh.add_driver( RandomPath(map.dim) );
        sensor = RangeBearingSensor(veh, map, 'covar', W);
        ekf_m = EKF(veh, [], [], sensor, W, []);
        ekf_m.run(t);

        map.plot();
        ekf_m.plot_map('g');
        veh.plot_xy('b');

        visualize({}, {}, [], map, veh, 'n', []);
        pause(1)


        % SLAM
        veh = Bicycle('covar', V);
        veh.add_driver( RandomPath(map.dim) );
        sensor = RangeBearingSensor(veh, map, 'covar', W);
        ekf_s = EKF(veh, V, P0, sensor, W, []);
        ekf_s.run(t);

        map.plot();
        ekf_s.plot_map('g');
        ekf_s.plot_xy('r');
        veh.plot_xy('b');


        % After creating a map and vehicle, and running the Robotics toolbox
        % EKF, running the following line should produce Figure 1.
        % If it does not, you may have forgotten to reset the random seed to 0.
        visualize({}, {}, [], map, veh, 'n', []);
        pause(1)
end