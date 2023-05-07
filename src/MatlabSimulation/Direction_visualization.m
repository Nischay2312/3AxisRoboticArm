% Create a 2D example with start, end, and obstacle (singularity) positions
start_pos = [0, 0];
end_pos = [10, 10];
obstacle_pos = [5, 5];

% Generate a straight path from start to end
n_points = 100;
path = linspace(0, 1, n_points)' * (end_pos - start_pos) + start_pos;

% Find the points close to the obstacle (within the threshold distance)
threshold_distance = 1.5;
dist_to_obstacle = vecnorm(path - obstacle_pos, 2, 2);
close_points_idx = find(dist_to_obstacle < threshold_distance);

% Modify the path to avoid the obstacle
modified_path = path;
direction = (end_pos - start_pos) / norm(end_pos - start_pos);
modified_path(close_points_idx, :) = path(close_points_idx, :) + threshold_distance * direction;

% Plot the paths and obstacle
figure;
plot(path(:, 1), path(:, 2), 'b-', 'LineWidth', 1);
hold on;
plot(modified_path(:, 1), modified_path(:, 2), 'r--', 'LineWidth', 1);
scatter(obstacle_pos(1), obstacle_pos(2), 100, 'kx', 'LineWidth', 2);
scatter(start_pos(1), start_pos(2), 100, 'go', 'LineWidth', 2);
scatter(end_pos(1), end_pos(2), 100, 'ro', 'LineWidth', 2);
legend('Original Path', 'Modified Path', 'Obstacle (Singularity)', 'Start', 'End', 'Location', 'Best');
xlabel('X');
ylabel('Y');
title('Path Visualization with Obstacle Avoidance');
axis equal;
grid on;
