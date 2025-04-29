function animate(x, time, theta) %input is a state vector

figure('Name', 'Dynamics Animation');
hold on
xlabel('Horizontal Distance (m)');
ylabel('Vertical Distance (m)');
title('Ball Trajectory');
hold on
xlim([-0.5,2]);
ylim([-1.0, 15]);

bRecord = 1;
if bRecord
    % Define video recording parameters
    Filename = 'sliding';
    v = VideoWriter(Filename, 'MPEG-4');
    open(v);
end

b = [];
p = [];

x_trajectory = x(:, 1);
y_trajectory = x(:, 2);

for t = 1:length(time)-1
    delete(b);
    delete(p);
    x_ground = -5:0.01:10;
    y_ground = @(l) 1+tan(-theta)*l;  % Equation for the slanted ground
    b = plot(x_ground, y_ground(x_ground),'Color',[152 182 236]./255,'LineWidth',5);
    grid on;
    p = scatter(x_trajectory(t),y_trajectory(t),100,'MarkerFaceColor',[255 207 203]./255,'MarkerEdgeColor','k');
    xlim([-0.5 2.5])
    ylim([-2 12])

    % Pause to control the speed of the animation
    pause(time(t+1) - time(t));  % You can adjust the pause duration based on your preference

    % Capture the frame for the video
    frame = getframe(gcf);
    writeVideo(v, frame);
end

if bRecord
    close(v);
end