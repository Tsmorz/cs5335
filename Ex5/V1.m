% Tony Smoragiewicz
% Ex5

% load image and video of textbook 
function V1()

        % load template
        img_rgb = imread('ex5_data/book-img.jpg');

        % convert to gray
        img_gray = rgb2gray(img_rgb);
        % shrink the template dimensions
        max_dim = 500;
        book = shrink(img_gray, max_dim);
        
        % load video
        v = VideoReader('ex5_data/book.webm');
        frames= read(v);
        [rw, co, ch, frm] = size(frames);

        % loop through video frames
        num = 20;
        frames = frames(:, :, :, 1:num);
         [~, ~, ~, num] = size(frames);

         sf0 = isurf(book);
        for i = 2:num
                % matches from book or previous frames
                if i == 2
                        frame1 = rgb2gray(frames(:, :, :, 1));
                        frame1 = shrink(frame1, max_dim);
                        sf1 = isurf(frame1);
                        [m1, corresp] = match(sf0, sf1, 'top', 100);
                        [H1, in] = m1.ransac(@homography, 4);
                else
                        H1 = H2;
                        frame1 = frame2;
                end

                % convert to gray
                frame2 = rgb2gray(frames(:,:,:,i));
                frame2 = shrink(frame2, max_dim);
                sf2 = isurf(frame2);
                [m2, corresp] = match(sf0, sf2, 'top', 100);
                [H2, in] = m2.ransac(@homography, 4);
        
                H = H1*inv(H2);
                R = H(1:2, 1:2)
                t = H(1:2, 3)

                f1 = figure(1);
                movegui(f1, 'northwest')
                homwarp(H1, book, 'full')

                f2 = figure(2);
                movegui(f2, 'west')
                imshow(frame1)
                drawnow

        end

        figure(3)
        hold on
        idisp({book, frame1})
        m1.subset(50).plot('g')

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