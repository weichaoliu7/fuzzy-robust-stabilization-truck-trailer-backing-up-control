close all;

% extract x and y coordinates
x1 = position(:, 6:2:12);  % x-coordinates of points 1, 2, 3, 4
y1 = position(:, 7:2:13);  % y-coordinates of points 1, 2, 3, 4

x2 = position(:, 18:2:end);  % x-coordinates of points 7, 8, 9, 10
y2 = position(:, 19:2:end);  % y-coordinates of points 7, 8, 9, 10

x3 = position(:, 14);  % x-coordinate of point 5
y3 = position(:, 15);  % y-coordinate of point 5

x4 = position(:, 16);  % x-coordinate of point 6
y4 = position(:, 17);  % y-coordinate of point 6

% plot graph with equal axis scale
figure;
axis equal;  % set equal aspect ratio for x-axis and y-axis
hold on;
num_colors = 10;
colors = parula(num_colors);
color_index = 1;

for i = 1:size(position, 1)
    if i == 1 || mod(i, 8500) == 0
        % connect points 1, 2, 3, 4, 1
        plot([x1(i, :), x1(i, 1)]', [y1(i, :), y1(i, 1)]', 'Color', colors(mod(color_index, num_colors) + 1, :)); 
        % connect points 7, 8, 9, 10, 7
        plot([x2(i, :), x2(i, 1)]', [y2(i, :), y2(i, 1)]', 'Color', colors(mod(color_index, num_colors) + 1, :)); 
        % connect points 5 and 6
        plot([x3(i) x4(i)]', [y3(i) y4(i)]', 'Color', colors(mod(color_index, num_colors) + 1, :));
        color_index = color_index + 1;
    end
end
hold off;
