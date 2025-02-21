%% DEFINE NEW LIMB MODEL %%
clear 
clc
close all 
% This is a 3DOF Antropomorphic arm 
% SWING
a2 = 0.3; 
a3 = 0.4; 
d1 = 0.1; 

a = [0, a2, a3];
d = [d1, 0, 0];
alpha = [-pi/2, 0, 0];
offset = [0, 0, 0];

% Generate the link object associated with the robot model (DH param); 
L1 = Link('d', d(1), 'a', a(1), 'alpha', alpha(1), 'offset', offset(1)); % coxa
L2 = Link('d', d(2), 'a', a(2), 'alpha', alpha(2), 'offset', offset(2)); % femur
L3 = Link('d', d(3), 'a', a(3), 'alpha', alpha(3), 'offset', offset(3)); %tibia

L = [L1, L2, L3];
Limb = SerialLink(L);
Limb.name = 'Limb';
Limb.gravity = [0; 0; 9.81]; % gravity acceleration vector expressed in the base frame 

%% DEFINE INITIAL ROBOT MODEL 
N_limb = 4; 
W = 0.45; 
L = 0.45; 
T_tool = trotx(pi/2)*troty(pi/2)*trotz(pi/2);
q0_contact_swing = [0,0,pi/2];
ROBOT = Robot_model(W, L, Limb,q0_contact_swing, [1 1 1 1], T_tool);

%% DEFINE  SCALING MATRIX 
q_dot_lim_i = [1.5, 2, 3];
q_dot_lim = [];
for i=1:N_limb
    q_dot_lim = [q_dot_lim, q_dot_lim_i];
end
Q_lim = diag(q_dot_lim);

%% INITIALIZE and PLOT
q0 = zeros(N_limb,Limb.n);
q0_contact = q0; 
for i=1:N_limb
    q0_contact(i,3) = pi/2;
end

figure('Name', 'Robot DH')
hold on 

% Plot Robot
plot_robot(ROBOT, q0_contact);

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

%h_base_ellipse = plot_ellipse(E_base(1:3,1:3),[T_base0(1,4), T_base0(2,4), T_base0(3,4)], 'r', 'alpha', 0.6);
h_base_ellipse = plotEllipsoidLines(E_base(1:3,1:3)^-1,[T_base0(1,4), T_base0(2,4), T_base0(3,4)], 'r');

%% LIMB MANIPULABILITY ELLIPSOID
% Initialization 
limbs_mask = [1 1 1 1]; % Visualize all limbs ellipsoid
% Initialize graphical element 
for i=1:sum(limbs_mask)
    h_limb_ellipses0{i} = plot_ellipse(eye(3));
end
[E_limbs, h_limb_ellipses] = limb_ellipsoids(ROBOT, q0_contact, limbs_mask, h_limb_ellipses0);
clear h_limb_ellipses0

pause

%% MOVE THE BASE 
x_motion = 0.1; 
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
[E_base, Ja] = compute_base_ellipsoid_scaled(ROBOT, q_new, grasp_matrix, T_base, Q_lim);
% Plot ellipsoid, in the base frame
delete(h_base_ellipse);
h_base_ellipse = plot_ellipse(E_base(1:3,1:3),[T_base(1,4), T_base(2,4), T_base(3,4)], 'r', 'alpha', 0.6);

%% LIMB MANIPULABILITY ELLIPSOID
% Initialization 
limbs_mask = [1 1 1 0]; % Visualize limbs ellipsoid
[E_limbs, h_limb_ellipses] = limb_ellipsoids(ROBOT, q_new, limbs_mask, h_limb_ellipses);

pause 

%% MOVE THE BASE AGAIN
x_motion = -0.1; 
y_motion = -0.1; 
z_motion = -0.1; 
T_base_in = T_base;
[q_new, T_base] = translate_base(ROBOT, T_base_in,  q_new, x_motion, y_motion, z_motion);
% Update graphics 
plot_robot(ROBOT, q_new);
[T_limb_root,r_base, h_root, h_base, h_base_poly, h_support, h_CoM] = update_frames(ROBOT,q_new, T_base, W, L, h_root, h_base, h_base_poly, h_support, h_CoM);

%% BASE MANIPULABILITY ELLIPSOID 
% Compute Grasp matrix and then Ellipsoid core
grasp_matrix = compute_grasp_matrix(r_base);
[E_base, Ja] = compute_base_ellipsoid_scaled(ROBOT, q_new, grasp_matrix, T_base, Q_lim);
% Plot ellipsoid, in the base frame
delete(h_base_ellipse);
%h_base_ellipse = plot_ellipse(E_base(1:3,1:3),[T_base(1,4), T_base(2,4), T_base(3,4)], 'r', 'alpha', 0.6);
h_base_ellipse = plotEllipsoidLines(E_base(1:3,1:3)^-1,[T_base(1,4), T_base(2,4), T_base(3,4)], 'r');


%% LIMB MANIPULABILITY ELLIPSOID
% Initialization 
limbs_mask = [1 0 0 0]; % Visualize limbs ellipsoid
[E_limbs, h_limb_ellipses] = limb_ellipsoids(ROBOT, q_new, limbs_mask, h_limb_ellipses);

pause
%% SEQUENTIAL MOTION 
for i=1:5
    pause
    % MOVE THE BASE AGAIN
    x_motion = -0.03; 
    y_motion = 0.0; 
    z_motion = 0.0; 
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
     h_base_ellipse = plotEllipsoidLines(E_base(1:3,1:3)^-1,[T_base(1,4), T_base(2,4), T_base(3,4)], 'r');
    

    % LIMB MANIPULABILITY ELLIPSOID
    % Initialization 
    limbs_mask = [1 1 1 1]; % Visualize limbs ellipsoid
    [E_limbs, h_limb_ellipses] = limb_ellipsoids(ROBOT, q_new, limbs_mask, h_limb_ellipses);
end


pause
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
h_base_ellipse = plot_ellipse(E_base(1:3,1:3),[T_base(1,4), T_base(2,4), T_base(3,4)], 'r', 'alpha', 0.6);

%% LIMB MANIPULABILITY ELLIPSOID
% Initialization 
limbs_mask = [1 1 1 1]; % Visualize limbs ellipsoid
[E_limbs, h_limb_ellipses] = limb_ellipsoids(ROBOT, q_new, limbs_mask, h_limb_ellipses);

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
     h_base_ellipse = plotEllipsoidLines(E_base(1:3,1:3)^-1,[T_base(1,4), T_base(2,4), T_base(3,4)], 'r');
    

    % LIMB MANIPULABILITY ELLIPSOID
    % Initialization 
    limbs_mask = [1 1 1 1]; % Visualize limbs ellipsoid
    [E_limbs, h_limb_ellipses] = limb_ellipsoids(ROBOT, q_new, limbs_mask, h_limb_ellipses);
end




