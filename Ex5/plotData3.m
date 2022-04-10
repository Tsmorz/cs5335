function [] = plotData3(data, marker)
        x = data(1,:);
        y = data(2,:);
        z = data(3,:);
        scatter3(x, y, z, marker)
end

