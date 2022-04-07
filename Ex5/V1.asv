% Tony Smoragiewicz
% Ex5

% load image and video of textbook 
function V1()

        img_rgb = imread('ex5_data/book-img.jpg');
        img_gray = rgb2gray(img_rgb);
        max_dim = 300;
        book = shrink(img_gray, max_dim);
        
        v = VideoReader('ex5_data/book.webm');
        frames= read(v);

        [r, c, ch, num] = size(frames);
        frames = frames(:, :, :, 1:18:num);
        frame = frames(:,:,:,1);
        frame = rgb2gray(frame);
        frame = shrink(frame, max_dim);

        sf0 = isurf(book);
        sf1 = isurf(frame);
        [m, corresp] = match(sf0, sf1, 'top', 20);

        figure(1)
        idisp({book, frame})
        m.subset(100).plot('w')

        [H, in] = m.ransac(@homography, 4);

        figure(2)
        homwarp(inv(H), frame, 'full')

        figure(3)
        homwarp(H, book, 'full')


end


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