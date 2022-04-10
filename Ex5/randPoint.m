%% Random point in data with real xyz value
function [point] = randPoint(data)

        [r, ~] = size(data);
        point = data(randi(r), :);

        while isnan(point)
                point = data(randi(r), :);
        end

end

