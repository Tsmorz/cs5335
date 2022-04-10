%% Load model
function [xyz, rgb] = loadScan(file)

        % Load Bunny
        data = load(file);
        rgb = data.ptcloud_rgb;
        xyz = data.ptcloud_xyz;
       
end

