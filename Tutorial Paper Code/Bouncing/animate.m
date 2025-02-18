function animate(x, time)

figure('Name', 'Bounce Animation');
hold on
title('Bouncing Ball');
ylabel('Height');
hold on
xlim([-0.5,0.5]);
ylim([-0.1,2]);

bRecord = 1;
if bRecord
    % Define video recording parameters
    Filename = 'sliding';
    v = VideoWriter(Filename, 'MPEG-4');
    open(v);
end

b = [];
p = [];
   
for j = 1:length(time)-1
    delete(b);
    delete(p);
    t = j;
    y = x(j,1);
    b = scatter(0,y,100,'MarkerFaceColor',[152 182 236]./255,'MarkerEdgeColor','k');
    p = plot([-0.75,0.75],[0,0],'Color',[255 207 203]./255,'LineWidth',5);
        
    % Pause to control the speed of the animation
    pause(time(t+1) - time(t));  % You can adjust the pause duration based on your preference

    % Capture the frame for the video
    frame = getframe(gcf);
    writeVideo(v, frame);
end

if bRecord
    close(v);
end