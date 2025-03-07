function h_ee = plot_EE(h_ee_in, T_world_ee, Mode)
% Given transformation matrix (4x4)
delete(h_ee_in);

T = T_world_ee.T;

% Define the circle in the local YZ plane
if Mode == "W"
    r = 0.06; % Wheel radius
elseif Mode == "G"
    r = 0.1; % Gripper radius
end

theta = linspace(0, 2*pi, 100);
y_local = r * cos(theta); % Y-coordinates
z_local = r * sin(theta); % Z-coordinates
x_local = zeros(size(theta)); % X remains 0 in local frame

% Create homogeneous coordinates (4xN matrix)
circle_local = [x_local; y_local; z_local; ones(1, length(theta))];

% Transform circle to the world frame
circle_transformed = T * circle_local;

% Extract transformed coordinates
x_transformed = circle_transformed(1, :);
y_transformed = circle_transformed(2, :);
z_transformed = circle_transformed(3, :);

% Plot the transformed circle
h_ee = plot3(x_transformed, y_transformed, z_transformed, 'r', 'LineWidth', 3);
axis equal
xlabel('X'); ylabel('Y'); zlabel('Z');

