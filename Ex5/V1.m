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
        [~, ~, ~, num] = size(frames);
        frames = frames(:, :, :, 1:10:num);

        sf0 = isurf(book);
         [~, ~, ~, num] = size(frames);
        for i = 2:num
                frame = rgb2gray(frames(:,:,:,i));
                frame = shrink(frame, max_dim);
                sf1 = isurf(frame);
                [m, corresp] = match(sf0, sf1, 'top', 50);
        
                [H, in] = m.ransac(@homography, 4);
                H
        
%                 figure(2)
%                 homwarp(inv(H), frame, 'full')
                homwarp(H, book, 'full')
                sf0 = isurf(frame);
        end
        figure(1)
        idisp({book, frame})
        m.subset(100).plot('w')

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