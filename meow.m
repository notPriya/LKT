if visualize
    mov = VideoWriter('../../VO Presentation/pipe_lkt_results.avi');
    mov.FrameRate = 7;
    open(mov);
    for i = start:start+n-1
        % Extract the image.
%         I = preprocessImage(frames(:,:,:,i+1), false, false);
        I = frames(:, :, :, i+1);

        % Determine the optimal affine transform.
        M = TrackedObject.M(:, :, i-start+1);
%         T = TrackedObject.template{i-start+1}.template;
%         T = uint8(reshape(T, size(I, 1), size(I, 2)));

        % Warp the image to fit the template.
        template = warp.doWarp(I, M);

        % Smash it back into an image.
%         template = uint8(template);

        % Show the template.
%         subplot(2, 1, 1);
        imshow(template);
%         title(i);
        
        % Show deviations from the original template.
%         subplot(2, 1, 2);
%         imagesc(abs(T - template));

%         pause(0.3);
        writeVideo(mov, getframe);
        close;
    end
    close(mov);
end
