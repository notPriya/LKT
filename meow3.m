n = size(frames, 4);

mov = VideoWriter('../../VO Presentation/snake_video.avi');
mov.FrameRate = 10;
open(mov);
for i=500:1000
   % Find Features in the image.
   imshow(frames(:, :, :, i));
   drawnow;
   writeVideo(mov, getframe);
end
close(mov);