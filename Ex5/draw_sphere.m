% Draw a sphere with specified center position and radius
function draw_sphere(position, radius)
    [X,Y,Z] = sphere;
    X = X * radius;
    Y = Y * radius;
    Z = Z * radius;
    X = X + position(1);
    Y = Y + position(2);
    Z = Z + position(3);
    surf(X,Y,Z);
end

