%% DEFINE NEW LIMB MODEL %%

clear 
clc
close all 

% This is a 3DOF  leg 
% SWING

%define different limbs model 
a_3 = [0.0, 0.20, 0.18];
a_2 = [0.0, 0.10, 0.08];
a_1 = [0.0, 0.08, 0.06];
alpha = [-pi/2, 0, 0];
d = [0,0,0];
offset = [0, 0, 0];

a = [a_1; a_2; a_3];
d = [d; d; d; d];
alpha = [alpha; alpha; alpha]; 
offset = [offset; offset; offset];

N_limb = 6; 

% Generate the link object associated with the robot model (DH param); 
for i=1:N_limb/2
    L1 = Link('d', d(i,1), 'a', a(i,1), 'alpha', alpha(i,1), 'offset', offset(i,1)); % coxa
    L2 = Link('d', d(i,2), 'a', a(i,2), 'alpha', alpha(i,2), 'offset', offset(i,2)); % femur
    L3 = Link('d', d(i,3), 'a', a(i,3), 'alpha', alpha(i,3), 'offset', offset(i,3)); %tibia
    L(i,:) = [ L1, L2, L3];
end


for i=1:N_limb/2
    Limbs(i,:) =  SerialLink(L(i,:));
    Limbs(i,:).name = ['Limb',num2str(i)];
    Limbs(i,:).gravity = [0; 0; 9.81]; % gravity acceleration vector expressed in the base frame 
end


W = 0.10; 
L = 0.50; 
T_tool = trotx(pi/2)*troty(pi/2)*trotz(pi/2);
limbs_place = [0.05, 0.15, 0.30];
limbs_angle = [pi/6, pi/2+pi/3, pi-pi/10];

q0 = [];
q0(1,:) = [0, -30*pi/180, (180-60)*pi/180];
q0(2,:) = [0, -30*pi/180, (180-60)*pi/180];
q0(3,:) = [0, -15*pi/180, (180-10)*pi/180];

ROBOT = Robot_model_general(W, L, Limbs, limbs_place,limbs_angle, q0, [1 1 1 1 1 1], T_tool);

%% INITIALIZE and PLOT
figure('Name', 'Robot DH')
hold on 

q0 = [q0; q0];
% Plot Robot
plot_robot(ROBOT, q0);

%% PLOT FRAMES 

% Initialize graphics elements
T_base0 = transl(0.15,0,ROBOT(1).base.t(3));

for i = 1:N_limb
    h_root0{i} = trplot(eye(4)); 
end
h_base0 = trplot(T_base0,'rgb', 'length', 0.1, 'arrow');
h_CoM0 = plot3(0,0,0);

zero_N = zeros(1, N_limb);
h_base_poly0 = plot3(zero_N,zero_N,zero_N);
h_support0 = plot3(zero_N, zero_N, zero_N);

% Create graphics 
[T_limb_root,r_base, h_root, h_base, h_base_poly, h_support, h_CoM] = update_frames(ROBOT, q0, T_base0, W, L, h_root0, h_base0, h_base_poly0, h_support0, h_CoM0);
clear h_root0 h_base0 h_CoM0 h_base_poly0 h_support0

    for i=1:N_limb
        delete(h_root{i}); % limbs root frames 
    end

%% LIMIT 
q_dot_lim_1 = [0.8, 0.8, 0.8];
q_dot_lim_2 = [0.8, 0.8, 0.8];
q_dot_lim_3 = [1.0, 1.0, 4.0];

q_dot_lim = [q_dot_lim_1, q_dot_lim_2, q_dot_lim_3, q_dot_lim_1, q_dot_lim_2, q_dot_lim_3];
Q_lim = diag(q_dot_lim);


%% BASE MANIPULABILITY ELLIPSOID 
% Compute Grasp matrix and then Ellipsoid core
grasp_matrix = compute_grasp_matrix(r_base);
[E_base, Ja] = compute_base_ellipsoid_scaled(ROBOT, q0, grasp_matrix, T_base0, Q_lim);
% Plot ellipsoid, in the base frame

%h_base_ellipse = plot_ellipse(E_base(1:3,1:3),[T_base0(1,4), T_base0(2,4), T_base0(3,4)], 'r', 'alpha', 0.6);
h_base_ellipse = plotEllipsoidLines(E_base(1:3,1:3)^-1,[T_base0(1,4), T_base0(2,4), T_base0(3,4)], 'r');

%% LIMB MANIPULABILITY ELLIPSOID
% Initialization 
limbs_mask = [1 1 1 1 1 1]; % Visualize all limbs ellipsoid
% Initialize graphical element 
for i=1:sum(limbs_mask)
    h_limb_ellipses0{i} = plot_ellipse(eye(3));
end
[E_limbs, h_limb_ellipses] = limb_ellipsoids_general_scaled(ROBOT, q0, limbs_mask, h_limb_ellipses0, Q_lim);
clear h_limb_ellipses0
