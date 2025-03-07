%% DEFINE NEW LIMB MODEL %%
clear 
clc
close all 

% SWING
load('limbero_data');

Limb = limbero; 
Limb.qlim = [-2.10, 2.10; -1.95, 1.30; -0.40, 2.95; -pi/2, pi/2; -pi, pi; -0.25*pi, 0.5*pi; -inf, inf];

%% DEFINE ROBOT MODEL 
N_limb = 4; 
W = 0.25; 
L = 0.25; 
T_tool = trotx(pi/2)*troty(pi/2)*trotz(pi/2);
q0_contact_swing = [0, -pi/6, pi/2+pi/6, 0, 0, 0, 0];
ROBOT = Robot_model(W, L, Limb, q0_contact_swing, [1 1 1 1], T_tool);

%% INITIALIZE and PLOT


q0 = zeros(N_limb,Limb.n);
q0_wheel = q0;
q0_contact = q0; 
for i=1:N_limb
    q0_contact(i,2) = -pi/6;
    q0_contact(i,3) = pi/2+pi/6;

    q0_contact(i,2) = -pi/6;
    q0_wheel(i,3) = pi/2 + pi/6;
    q0_wheel(i,5) = pi;
    q0_wheel(i,6) = pi/2;
end

q0_wheel(1,5) = pi-pi/4;
q0_wheel(2,5) = pi+pi/4;
q0_wheel(3,5) = pi-pi/4;
q0_wheel(4,5) = pi+pi/4;

wheel = 0;
if wheel == 1
    q0_contact = q0_wheel;
end

figure('Name', 'Robot DH')
hold on 

% Plot Robot
plot_robot(ROBOT, q0_contact);

%% DEFINE  SCALING MATRIX 
q_dot_lim_i = [3.37, 3.37, 3.88, 3.88, 4.47, 4.47, 3.88 ];
q_dot_lim = [];
for i=1:N_limb
    q_dot_lim = [q_dot_lim, q_dot_lim_i];
end
Q_lim = diag(q_dot_lim);

%% PLOT FRAMES 

% Initialize graphics elements
T_base0 = transl(0,0,ROBOT(1).base.t(3));

for i = 1:N_limb
    h_root0{i} = trplot(eye(4)); 
end
h_base0 = trplot(T_base0,'rgb', 'length', 0.1, 'arrow');
h_CoM0 = plot3(0,0,0);
h_base_poly0 = plot3([0 0 0 0 0],[0 0 0 0 0],[0 0 0 0 0]);
h_support0 = plot3([0 0 0 0 0],[0 0 0 0 0],[0 0 0 0 0]);

% Create graphics 
[T_limb_root,r_base, h_root, h_base, h_base_poly, h_support, h_CoM] = update_frames(ROBOT, q0_contact, T_base0, W, L, h_root0, h_base0, h_base_poly0, h_support0, h_CoM0);
clear h_root0 h_base0 h_CoM0 h_base_poly0 h_support0

%% BASE MANIPULABILITY ELLIPSOID 
% Compute Grasp matrix and then Ellipsoid core
grasp_matrix = compute_grasp_matrix(r_base);
[E_base, Ja] = compute_base_ellipsoid_scaled(ROBOT, q0_contact, grasp_matrix, T_base0, Q_lim);
% Plot ellipsoid, in the base frame
% h_base_ellipse = plot_ellipse(E_base(1:3,1:3),[T_base0(1,4), T_base0(2,4), T_base0(3,4)], 'r', 'alpha', 0.6);
 h_base_ellipse = plotEllipsoidLines(9*E_base(1:3,1:3)^-1,[T_base0(1,4), T_base0(2,4), T_base0(3,4)], 'r');

%% LIMB MANIPULABILITY ELLIPSOID
% Initialization 
limbs_mask = [1 1 1 1]; % Visualize all limbs ellipsoid
% Initialize graphical element 
for i=1:sum(limbs_mask)
    h_limb_ellipses0{i} = plot_ellipse(eye(3));
end
[E_limbs, h_limb_ellipses] = limb_ellipsoids_general_scaled(ROBOT, q0_contact, limbs_mask, h_limb_ellipses0, Q_lim);
clear h_limb_ellipses0


disp(" press ENTER to Start GRIEEL Locomotion transformation")
pause();

Mode = "W";
[q_new, T_base, T_limb_root,r_base, h_root, h_base, h_base_poly, h_support, h_CoM] = GRIEEL_transform(ROBOT, Mode,  W, L, q0_contact, T_base0, T_limb_root, r_base, h_root, h_base, h_base_poly, h_support, h_CoM, Q_lim);

pause();

%% MOVE THE BASE 
x_motion = -0.2; 
y_motion = 0.1; 
z_motion = 0.0; 
T_base_in = T_base0;
[q_new, T_base] = translate_base(ROBOT, T_base_in,  q0_contact, x_motion, y_motion, z_motion);
% Update graphics 
plot_robot(ROBOT, q_new);
[T_limb_root,r_base, h_root, h_base, h_base_poly, h_support, h_CoM] = update_frames(ROBOT,q_new, T_base, W, L, h_root, h_base, h_base_poly, h_support, h_CoM);

%% BASE MANIPULABILITY ELLIPSOID 
% Compute Grasp matrix and then Ellipsoid core
grasp_matrix = compute_grasp_matrix(r_base);
[E_base, Ja] = compute_base_ellipsoid_scaledv2(ROBOT, q_new, grasp_matrix, T_base, Q_lim);
% Plot ellipsoid, in the base frame
delete(h_base_ellipse);
% h_base_ellipse = plot_ellipse(E_base(1:3,1:3),[T_base(1,4), T_base(2,4), T_base(3,4)], 'r', 'alpha', 0.6);
h_base_ellipse = plotEllipsoidLines(10*E_base(1:3,1:3)^-1,[T_base(1,4), T_base(2,4), T_base(3,4)], 'r');

%% LIMB MANIPULABILITY ELLIPSOID
% Initialization 
limbs_mask = [1 1 1 1]; % Visualize limbs ellipsoid
[E_limbs, h_limb_ellipses] = limb_ellipsoids_general_scaled(ROBOT, q_new, limbs_mask, h_limb_ellipses, Q_lim);

pause 

disp(" ")
%% MOVE THE BASE AGAIN
x_motion = 0.2; 
y_motion = -0.1; 
z_motion = 0.0; 
T_base_in = T_base;
[q_new, T_base] = translate_base(ROBOT, T_base_in,  q_new, x_motion, y_motion, z_motion);
% Update graphics 
plot_robot(ROBOT, q_new);
[T_limb_root,r_base, h_root, h_base, h_base_poly, h_support, h_CoM] = update_frames(ROBOT,q_new, T_base, W, L, h_root, h_base, h_base_poly, h_support, h_CoM);

%% BASE MANIPULABILITY ELLIPSOID 
% Compute Grasp matrix and then Ellipsoid core
grasp_matrix = compute_grasp_matrix(r_base);
[E_base, Ja] = compute_base_ellipsoid_scaledv2(ROBOT, q_new, grasp_matrix, T_base, Q_lim);
% Plot ellipsoid, in the base frame
delete(h_base_ellipse);
% h_base_ellipse = plot_ellipse(E_base(1:3,1:3),[T_base(1,4), T_base(2,4), T_base(3,4)], 'r', 'alpha', 0.6);
h_base_ellipse = plotEllipsoidLines(10*E_base(1:3,1:3)^-1,[T_base(1,4), T_base(2,4), T_base(3,4)], 'r');

%% LIMB MANIPULABILITY ELLIPSOID
% Initialization 
limbs_mask = [1 1 1 1]; % Visualize limbs ellipsoid
[E_limbs, h_limb_ellipses] = limb_ellipsoids_general_scaled(ROBOT, q_new, limbs_mask, h_limb_ellipses, Q_lim);

% SEQUENTIAL MOTION 
for i=1:15
    % MOVE THE BASE AGAIN
    x_motion = 0.0; 
    y_motion = 0.0; 
    z_motion = 0.01; 
    T_base_in = T_base;
    [q_new, T_base] = translate_base(ROBOT, T_base_in,  q_new, x_motion, y_motion, z_motion);
    % Update graphics 
    plot_robot(ROBOT, q_new);
    [T_limb_root,r_base, h_root, h_base, h_base_poly, h_support, h_CoM] = update_frames(ROBOT,q_new, T_base, W, L, h_root, h_base, h_base_poly, h_support, h_CoM);

    % BASE MANIPULABILITY ELLIPSOID 
    % Compute Grasp matrix and then Ellipsoid core
    grasp_matrix = compute_grasp_matrix(r_base);
    [E_base, Ja] = compute_base_ellipsoid_scaledv2(ROBOT, q_new, grasp_matrix, T_base, Q_lim);
    % Plot ellipsoid, in the base frame
    delete(h_base_ellipse);
    % h_base_ellipse = plot_ellipse(E_base(1:3,1:3),[T_base(1,4), T_base(2,4), T_base(3,4)], 'r', 'alpha', 0.6);
    h_base_ellipse = plotEllipsoidLines(10*E_base(1:3,1:3)^-1,[T_base(1,4), T_base(2,4), T_base(3,4)], 'r');

    m = sqrt(det(E_base));
    disp("Base Manipulability: " + num2str(real(m)));

    % LIMB MANIPULABILITY ELLIPSOID
    % Initialization 
    limbs_mask = [1 1 1 1]; % Visualize limbs ellipsoid
    [E_limbs, h_limb_ellipses] = limb_ellipsoids_general_scaled(ROBOT, q_new, limbs_mask, h_limb_ellipses, Q_lim);
    disp(" ")
end

pause

disp(" ")
%% RISE ROBOT LF LIMB 
i_rise = 1; % 1=LF, 2=LH, 3=RH, 4=RF
limb_names = ["LF*"; "LH*"; "RH*"; "RF*"];
ROBOT(i_rise).name = limb_names(i_rise, :);

close all
figure('Name','Robot DH')
hold on 
q_new = move_limb(ROBOT, q_new, i_rise, 0, 0, 0.15);
plot_robot(ROBOT, q_new);

[T_limb_root,r_base, h_root, h_base, h_base_poly, h_support, h_CoM] = update_frames(ROBOT,q_new, T_base, W, L, h_root, h_base, h_base_poly, h_support, h_CoM);

%% BASE MANIPULABILITY ELLIPSOID 
% Compute Grasp matrix and then Ellipsoid core
grasp_matrix = compute_grasp_matrix(r_base);
[E_base, Ja] = compute_base_ellipsoid_scaled(ROBOT, q_new, grasp_matrix, T_base, Q_lim);
% Plot ellipsoid, in the base frame
delete(h_base_ellipse);
 h_base_ellipse = plotEllipsoidLines(10*E_base(1:3,1:3)^-1,[T_base(1,4), T_base(2,4), T_base(3,4)], 'r');

%% LIMB MANIPULABILITY ELLIPSOID
% Initialization 
limbs_mask = [1 1 1 1]; % Visualize limbs ellipsoid
[E_limbs, h_limb_ellipses] = limb_ellipsoids_general_scaled(ROBOT, q_new, limbs_mask, h_limb_ellipses, Q_lim);

pause
disp(" ")
% SEQUENTIAL MOTION 
for i=1:5
    pause
    % MOVE THE BASE AGAIN
    x_motion = -0.02; 
    y_motion = -0.02; 
    z_motion = 0.02; 
    T_base_in = T_base;
    [q_new, T_base] = translate_base(ROBOT, T_base_in,  q_new, x_motion, y_motion, z_motion);
    % Update graphics 
    plot_robot(ROBOT, q_new);
    [T_limb_root,r_base, h_root, h_base, h_base_poly, h_support, h_CoM] = update_frames(ROBOT,q_new, T_base, W, L, h_root, h_base, h_base_poly, h_support, h_CoM);
    
    % BASE MANIPULABILITY ELLIPSOID 
    % Compute Grasp matrix and then Ellipsoid core
    grasp_matrix = compute_grasp_matrix(r_base);
    [E_base, Ja] = compute_base_ellipsoid_scaled(ROBOT, q_new, grasp_matrix, T_base, Q_lim);
    % Plot ellipsoid, in the base frame
     delete(h_base_ellipse);
%    h_base_ellipse = plot_ellipse(E_base(1:3,1:3),[T_base(1,4), T_base(2,4), T_base(3,4)], 'r', 'alpha', 0.6);
    h_base_ellipse = plotEllipsoidLines(10*E_base(1:3,1:3)^-1,[T_base(1,4), T_base(2,4), T_base(3,4)], 'r');
    

    % LIMB MANIPULABILITY ELLIPSOID
    % Initialization 
    limbs_mask = [1 1 1 1]; % Visualize limbs ellipsoid
    [E_limbs, h_limb_ellipses] = limb_ellipsoids_general_scaled(ROBOT, q_new, limbs_mask, h_limb_ellipses, Q_lim);
end