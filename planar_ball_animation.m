function []=planar_ball_animation(fps, name, t_all, q_all, xy_b)

num_frames = numel(t_all);
% fps = 15;
r = 0.1;

v = VideoWriter(name,'MPEG-4');
    v.FrameRate = fps;
    open(v);
figure
for t_anime = 0:1/fps:t_all(end)

    q_t = interp1(t_all(1:10:end), q_all(1:10:end, :), t_anime);
    [x_P1,y_P1,x_P2,y_P2,x_Ball,y_Ball] = state2visiual(q_t);


    x1 = [0, x_P1];
    y1 = [0, y_P1];
    plot(x1, y1, 'b-', 'LineWidth', 2);
    hold on;

%     % Plot the second link
    x2 = [x_P1, x_P2];
    y2 = [y_P1, y_P2];
    plot(x2, y2, 'b-', 'LineWidth', 2);

%     % Plot the ball
    plot(xy_b(1), xy_b(2), 'ob', 'MarkerSize', 300*r, 'MarkerFaceColor','None');
    plot(x_Ball, y_Ball, 'or', 'MarkerSize', 300*r, 'MarkerFaceColor', 'r');
    % circle(x_Ball,y_Ball,r);
    axis equal;
    xlim([-2, 2]);
    ylim([-2, 2]);
    % title(sprintf('Time: %.2f s', t(i)));
    xlabel('X (m)');
    ylabel('Y (m)');
%     
    % Update the plot
    drawnow;
    frame = getframe(gcf);
    pause(1/fps)
    writeVideo(v,frame);

    % Clear the plot
    clf;

end
close(v);
end

function [x_P1,y_P1,x_P2,y_P2,x_ball,y_ball] = state2visiual(q_t)
%     yout_cell = num2cell(yout);   
%     [x_ball,dx,y_ball,dy,lb,dlb,theta1,theta2] = deal(yout_cell{:});
    x_ball = q_t(1);
    y_ball = q_t(3);
    lb = q_t(5);
    theta1 = q_t(7);
    theta2 = q_t(9);
    % [x_ball,dx,y_ball,dy,lb,dlv,theta1,theta2] = yout(:);
    L1 = 1.0;
    L2 = 1.0;
    r = 0.1;
    % P1
    x_P1 = L1 * cos(theta1);
    y_P1 = L1 * sin(theta1);
     
    beta = theta1 + theta2 - pi;
    % P2 (only for visualization)
    x_P2 = x_P1 + L2 * cos(beta);
    y_P2 = y_P1 + L2 * sin(beta);
    % 
    % x_Ball = x_c - r * sin(beta);
    % y_Ball = y_c + r * cos(beta);
    
end