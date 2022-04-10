%% Unwrap sensor matrix to a row
function row_vec = unwrap(data)
        [r, c, ~] = size(data);
        x = data(:,:, 1);
        x = reshape(x, [r*c, 1]);
        
        y = data(:,:, 2);
        y = reshape(y, [r*c, 1]);
        
        z = data(:,:, 3);
        z = reshape(z, [r*c, 1]);

        row_vec = [x, y, z];

end
