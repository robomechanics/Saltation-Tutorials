function animate(states, dt)
    % Parameters
    
    % Sample data (replace this with your actual state data)
    N = size(states, 1);  % Number of time steps
    theta = states(:, 1);
    
    % Create video writer object
    videoFile = 'bouncing_animation.mp4';
    writerObj = VideoWriter(videoFile, 'MPEG-4');
    open(writerObj);
    
    % Create figure
    figure;
    mass = [];
    ground = [];
    
    for i = 1:N
        drawnow limitrate
        disp(["iteration: ", num2str(i)])

        % Plot pendulum
        x = 0;
        y = theta(i);
        delete(mass)
        delete(ground)

        ground = plot([-1 1], [0, 0], 'k', 'LineWidth', 2);
        % string = plot([0, x], [0, y], 'k', 'LineWidth', 2);
        hold on
        mass = scatter(x, y, 200, 'MarkerFaceColor', [247, 114, 174]/255, 'MarkerEdgeColor', [0, 0, 0]);
        axis equal;
        xlim([-1 1])
        ylim([-0.5 5])
        title(['Time (s): ', num2str((i-1)*dt)]);
        xlabel('x');
        ylabel('y');
        grid on;
    
        % Capture frame for video
        frame = getframe(gcf);
        writeVideo(writerObj, frame);
    end
    
    % Close video writer
    close(writerObj);
end