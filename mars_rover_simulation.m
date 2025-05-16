% Mars Rover Simulation: LiDAR, DEM, Point Cloud, and Grid Map
clear all; close all; clc;

% Step 2: Load and preprocess the Mars surface image
img = imread('mars surface.png'); % Replace with your image file name, the image is of Mar's Jezero Crater
if size(img, 3) == 3
    img = rgb2gray(img); % Convert to grayscale if RGB
end
img = imresize(img, [100, 100]); % Resize to 100x100 pixels for simplicity
height_map = double(img) / 255; % Normalize to [0,1] for elevation

% Visualize the height map
figure;
imshow(img);
title('Mars Surface Height Map');

% Step 3: Simulate LiDAR scanning
rover_pos = [50, 50]; % Rover position (center of 100x100 grid)
max_range = 20; % LiDAR maximum range (pixels)
angles = linspace(0, 2*pi, 360); % 360-degree scan
lidar_distances = zeros(1, length(angles));
lidar_points = [];

for i = 1:length(angles)
    theta = angles(i);
    for r = 1:max_range
        x = round(rover_pos(1) + r * cos(theta));
        y = round(rover_pos(2) + r * sin(theta));
        if x > 0 && x <= 100 && y > 0 && y <= 100
            height_diff = height_map(x, y) - height_map(rover_pos(1), rover_pos(2));
            if height_diff > 0.1 % Detect objects with significant height
                lidar_distances(i) = r;
                lidar_points = [lidar_points; x, y, height_map(x, y)]; % Store x, y, and elevation
                break;
            end
        else
            lidar_distances(i) = r;
            break;
        end
    end
end

% Visualize LiDAR scan
figure;
polarplot(angles, lidar_distances);
title('LiDAR Scan around Rover');

% Step 4: Create Digital Elevation Model (DEM)
dem = imgaussfilt(height_map, 2); % Smooth the height map
[X, Y] = meshgrid(1:100, 1:100);

% Visualize DEM as 3D surface
figure;
surf(X, Y, dem, 'EdgeColor', 'black');
colormap('jet');
colorbar;
title('Digital Elevation Model of Mars Surface');
xlabel('X (pixels)'); ylabel('Y (pixels)'); zlabel('Elevation');

% Step 5: Visualize Simulated LiDAR Point Cloud
figure;
if ~isempty(lidar_points)
    scatter3(lidar_points(:, 1), lidar_points(:, 2), lidar_points(:, 3), 10, lidar_points(:, 3), 'filled');
    colormap('jet');
    xlabel('X (pixels)');
    ylabel('Y (pixels)');
    zlabel('Elevation');
    title('Simulated LiDAR Point Cloud');
    grid on;
    axis([0 100 0 100 0 1]); % Match axis limits to grid and elevation range
else
    disp('No LiDAR points detected. Adjust height_diff threshold.');
end

% Step 6: Form Grid Map
grid_map = zeros(100, 100); % 0 = navigable, 1 = obstacle
[dx, dy] = gradient(dem); % Compute slopes
slope = sqrt(dx.^2 + dy.^2);

% Mark obstacles based on slope and LiDAR points
for i = 1:100
    for j = 1:100
        if slope(i, j) > 0.25 || dem(i, j) > 0.85 % Steep slopes or high elevation
            grid_map(i, j) = 1;
        end
    end
end
for k = 1:size(lidar_points, 1)
    x = lidar_points(k, 1); y = lidar_points(k, 2);
    if x > 0 && x <= 100 && y > 0 && y <= 100
        grid_map(x, y) = 1;
    end
end

% Visualize Grid Map
figure;
imshow(grid_map, []);
hold on;
plot(rover_pos(2), rover_pos(1), 'r*', 'MarkerSize', 10); % Rover position
title('Grid Map (0 = Navigable, 1 = Obstacle)');