% Tony Smoragiewicz
% Ex5 V1 - real time

close all

% load book image
img_rgb = imread('ex5_data/book-img.jpg');
img_gray = rgb2gray(img_rgb);
max_dim = 400;
book = shrink(img_gray, max_dim);
sf0 = isurf(book);

cam = webcam(1);

for idx = 1:10

        % Acquire a single image.
        rgbImage = snapshot(cam);
        
        % Convert RGB to grayscale.
        grayImage = rgb2gray(rgbImage);

        % reduce image res
        img = shrink(grayImage, max_dim);

        sf1 = isurf(img);
        [m, ~] = match(sf0, sf1, 'top', 50);
        
        [H, in] = m.ransac(@homography, 5);
         
        if in>0.5
                img = homwarp(H, book, 'full');
        end

        % Display the image.
        imshow(img);
        drawnow

end

clear('cam')

%% resize picture
function small = shrink(original, max_dim)

        [r, ~, ~] = size(original);
        if r>200
                scale = max_dim/r;
                small = imresize(original, scale);
        else
                small = original;
        end

end