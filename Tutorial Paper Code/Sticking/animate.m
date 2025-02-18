function animate(x, theta) %input is a state vector

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
    Filename = 'sticking';
    v = VideoWriter(Filename, 'MPEG-4');
    v.Quality = 100;
    v.FrameRate = 60;
    open(v);
end

b = [];
p = [];

x_trajectory = x(:, 1);
y_trajectory = x(:, 2);

for j = 1:size(x,1)
    delete(b);
    delete(p);
    x_ground = -5:0.01:10;
    y_ground = @(l) 1+tan(theta)*l;  % Equation for the slanted ground
    b = plot(x_ground, y_ground(x_ground),'Color',[152 182 236]./255,'LineWidth',5);
    grid on;
    p = scatter(x_trajectory(j),y_trajectory(j),100,'MarkerFaceColor',[255 207 203]./255,'MarkerEdgeColor','k');
        
    if bRecord
        frame = getframe(gcf);
        writeVideo(v,frame);
    end
end

if bRecord
    close(v);
end