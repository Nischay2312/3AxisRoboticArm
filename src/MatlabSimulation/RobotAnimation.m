% %animate the robot to read the path
    % Animation settings
    n_steps = size(JointArray,1); % Number of steps in the animation
    pause_duration = 0.051; % Pause duration between steps (in seconds)

    % Visualize the robot arm moving from the initial to the final joint angles
    figure(3)   
    config_initial = homeConfiguration(robot);
    for i = 1:num_links-1
        config_initial(i).JointPosition = JointArray(1,i);
    end
    ax = show(robot, config_initial);
    ax.XLim = [-1, 1]; ax.YLim = [-1, 1]; ax.ZLim = [-1, 1]; % Set axis limits
    ax.DataAspectRatio = [1 1 1];
    title('Robot Arm Animation');

    % Initialize arrays to store end-effector positions 
    end_effector_positions = zeros(n_steps, 3);

    for step = 1:n_steps
        config_intermediate = homeConfiguration(robot);
        for i = 1:num_links-1
            config_intermediate(i).JointPosition = JointArray(step,i);
        end
        % Store end-effector position
        T_endeffector = getTransform(robot, config_intermediate, robot.BodyNames{end});
        position_endeffector = tform2trvec(T_endeffector);
        end_effector_positions(step, :) = position_endeffector;
        show(robot, config_intermediate, 'Parent', ax);
        pause(pause_duration);
    end

    % Add labels for each link and the end-effector
    hold on;
    plot3(end_effector_positions(:,1), end_effector_positions(:,2), end_effector_positions(:,3), 'b.-', 'MarkerSize', 10);
    for i = 1:(num_links)
        body = robot.BodyNames{i};
        T = getTransform(robot, config_intermediate, body);
        position = tform2trvec(T);
        if i <= num_links
            text(position(1), position(2), position(3), ['Link', num2str(i)], 'FontSize', 12, 'Color', 'red', 'FontWeight', 'bold');
        else
            text(position(1), position(2), position(3), 'EndEffector', 'FontSize', 12, 'Color', 'red', 'FontWeight', 'bold');
        end
    end
    hold off;
