close all 
clear 
clc

%%%%%%%%%%%%%%%%%%%%%%%%%%
%% LIMBERO+GRIEEL ROBOT %%
%%%%%%%%%%%%%%%%%%%%%%%%%%
% Tentatives to make the code cleaner than limberoGrieel_robot_DH.m

% Load contact and swing limb model 
load('limbero_data');

% Define useful parameters
N_limb = 4; 
N_link = limbero.n;
W = 0.35; % Base width
L = 0.35; % Base Length

g = [0; 0; 9.81]; % Gravity vector


%%%%%%%%%%%%%%%%%%%%%%
%% SWING LEGS MODEL %%
%%%%%%%%%%%%%%%%%%%%%%

% Homogeneus Transformations for Base to Limbroot position
q_contact0 = [0,0,pi/2,0,0,0,0];
tz_ee = limbero.fkine(q_contact0).t(3); % z-coordinate of limb tool = limb height

T_LF = transl(L/2, W/2, -tz_ee)*trotz(pi/4*180/pi);
T_LH =  transl(-L/2, W/2, -tz_ee)*trotz((pi/2+pi/4)*180/pi);
T_RH =  transl(-L/2, -W/2, -tz_ee)*trotz((-pi/2-pi/4)*180/pi);
T_RF =  transl(L/2, -W/2, -tz_ee)*trotz(-pi/4*180/pi);
% Leg redefinition
LF_leg = SerialLink(limbero, 'name', 'LF', 'gravity', g, 'base', T_LF);
LH_leg = SerialLink(limbero, 'name', 'LH', 'gravity', g, 'base', T_LH);
RH_leg = SerialLink(limbero, 'name', 'RH', 'gravity', g, 'base', T_RH);
RF_leg = SerialLink(limbero, 'name', 'RF', 'gravity', g, 'base', T_RF);

ROBOT_SWING = [LF_leg, LH_leg, RH_leg, RF_leg]; 

q0 = zeros(1,N_link);
q1 = [0,0,pi/2,0,0,0,0];
for i=1:N_limb
    q(i,:) = q1; 
end

% Plot the robot 
figure('Name', 'LIMBERO+GRIEEL q=qz')
hold on
plot_robot(ROBOT_SWING, q);

% Represent Base Frame and Shape (as Rectangular patch) 
% I take as inertial frame the base-footprint center in the zero configuration
T_base_footprint = eye(4,4);
trplot(T_base_footprint, 'rgb', 'length', 0.1, 'arrow');
% Initialize Base Homogeneus transformation
T_base0 = transl(0,0,ROBOT_SWING(1).base.t(3));

% Initialize graphic handlers
for i = 1:N_limb
    h_root0{i} = trplot(eye(4)); 
end
h_base0 = trplot(T_base0,'rgb', 'length', 0.1, 'arrow');
h_patch0 = patch([0 0 0 0], [0 0 0 0], 'b');
% Plot other useful frames (limb root, base) and base patch
[T_limb_root, r_base, h_root, h_base, h_patch] = update_frames_contact(ROBOT_SWING, q, T_base0, W,L, h_root0, h_base0, h_patch0);
title('Robot Contact configuration, Swing limbs');

pause % Wait for user signal 
%% Experiment with Base motion
% 1) Translate the base
% 2) Plot the new limbs configuration
% 3) Move the useful frames and base patch
[q1,T_base] = translate_base(ROBOT_SWING,T_base0, q, 0.0, -0.1, 0.0);
plot_robot(ROBOT_SWING, q1);
[T_limb_root, r_base, h_root, h_base, h_patch] = update_frames_contact(ROBOT_SWING, q1, T_base, W,L, h_root, h_base, h_patch);

hold off

pause

%%%%%%%%%%%%%%%%%%%%%%%%
%% CONTACT LEGS MODEL %% 
%%%%%%%%%%%%%%%%%%%%%%%%
g = [0; 0; 9.81];             % Gravity vector
q0_contact = zeros(1,N_link); % zero joint configuration

% Homogeneus Transformations for Base to Limbroot position
tx_ee_contact = limbero_contact.fkine(q0_contact).t(1); % x-coordinate of limb tool = limb length
tz_ee_contact = limbero_contact.fkine(q0_contact).t(3); % z-coordinate of limb tool = limb height
t = tx_ee_contact*sqrt(2)/2; % transaltion to move limb base to correct position

T_LF_contact = transl(L/2+t, W/2+t, tool_length)*trotz(-(pi/2+pi/4)*180/pi)*limbero_contact.base.T;
T_LH_contact =  transl(-L/2-t, W/2+t, tool_length)*trotz(-(pi/4)*180/pi)*limbero_contact.base.T;
T_RH_contact =  transl(-L/2-t, -W/2-t, tool_length)*trotz((pi/4)*180/pi)*limbero_contact.base.T;
T_RF_contact =  transl(L/2+t, -W/2-t, tool_length)*trotz((pi/4+pi/2)*180/pi)*limbero_contact.base.T;

T_tool = trotz(pi*180/pi); % rotate the tool frame as in the URDF

% Leg redefinition
LF_leg_contact = SerialLink(limbero_contact, 'name', 'LF_{contact}', 'gravity', g, 'base', T_LF_contact, 'tool', T_tool);
LH_leg_contact = SerialLink(limbero_contact, 'name', 'LH_{contact}', 'gravity', g, 'base', T_LH_contact, 'tool', T_tool);
RH_leg_contact = SerialLink(limbero_contact, 'name', 'RH_{contact}', 'gravity', g, 'base', T_RH_contact, 'tool', T_tool);
RF_leg_contact = SerialLink(limbero_contact, 'name', 'RF_{contact}', 'gravity', g, 'base', T_RF_contact, 'tool', T_tool);

ROBOT_CONTACT = [LF_leg_contact, LH_leg_contact, RH_leg_contact, RF_leg_contact]; 

for i = 1:N_limb
    ROBOT_CONTACT(i).qlim(1,:) = [0 0]; % fix the driving joint to avoid using it 
    q_contact(i,:) = q0_contact;        % collect in a N_limb x N_joint matrix
end

% Plot the robot 
figure('Name', 'LIMBERO+GRIEEL CONTACT q=qz')
hold on
plot_robot(ROBOT_CONTACT, q_contact);

% Represent Base Frame and Shape (as Rectangular patch) 
% I take as inertial frame the base-footprint center in the zero configuration
T_base_footprint = eye(4,4);
trplot(T_base_footprint, 'rgb', 'length', 0.1, 'arrow');
% Initialize Base Homogeneus transformation
T_base0 = transl(0,0,ROBOT_CONTACT(1).fkine(q_contact(1,:)).t(3));

% Initialize graphic handlers
for i = 1:N_limb
    h_root0{i} = trplot(eye(4)); 
end
h_base0 = trplot(T_base0,'rgb', 'length', 0.1, 'arrow');
h_patch0 = patch([0 0 0 0], [0 0 0 0], 'b');

% Plot other useful frames (limb root, base) and base patch
[T_limb_root, r_base, h_root, h_base, h_patch] = update_frames_contact(ROBOT_CONTACT, q_contact, T_base0, W,L, h_root0, h_base0, h_patch0);

%% Experiment with Base manipulability Ellipsoid 
% Compute Grasp matrix and then Ellipsoid core
grasp_matrix = compute_grasp_matrix(r_base);
[E_base, Ja] = compute_base_ellipsoid(ROBOT_CONTACT, q_contact, grasp_matrix);
% Plot ellipsoid, in the base frame
h_ellipse = plot_ellipse(E_base(1:3,1:3),[T_base0(1,4), T_base0(2,4), T_base0(3,4)], 'r', 'alpha', 0.6);

pause % Wait for user signal 
%% Experiment with Base motion
% 1) Translate the base
% 2) Plot the new limbs configuration
% 3) Move the useful frames and base patch
[q1_contact,T_base] = translate_base(ROBOT_CONTACT,T_base0, q_contact, 0.0, -0.15, 0.0);
plot_robot(ROBOT_CONTACT, q1_contact);
[T_limb_root, r_base, h_root, h_base, h_patch] = update_frames_contact(ROBOT_CONTACT, q1_contact, T_base, W,L, h_root, h_base, h_patch);

% Update Base Ellipsoid
% Compute Grasp matrix and then Ellipsoid core
grasp_matrix = compute_grasp_matrix(r_base);
[E_base, Ja] = compute_base_ellipsoid(ROBOT_CONTACT, q_contact, grasp_matrix);
% Plot ellipsoid, in the base frame, removing the previous one
delete(h_ellipse);
h_ellipse = plot_ellipse(E_base(1:3,1:3),[T_base(1,4), T_base(2,4), T_base(3,4)], 'r', 'alpha', 0.6);

pause

%%%%%%%%%%%%%%%%%%%%%%%%%%
%% CONTACT & SWING LMBS %%
%%%%%%%%%%%%%%%%%%%%%%%%%%
close all

ROBOT = Robot_model(limbero,limbero_contact, [0 1 1 1], tool_length+0.02);

q0_swing = [0, 0, pi/2, 0, 0, 0, 0];
q0_contact = zeros(1,N_link);
contacts = check_contact_limbs(ROBOT);
for i=1:N_limb
    if contacts(i) == 1
        q(i,:) = q0_contact;
    elseif contacts(i) == 0
        q(i,:) = q0_swing;
    end
end
% Plot the robot 
figure('Name', 'LIMBERO+GRIEEL q=qz')
hold on
plot_robot(ROBOT, q);

% Represent Base Frame and Shape (as Rectangular patch) 
% I take as inertial frame the base-footprint center in the zero configuration
T_base_footprint = eye(4,4);
trplot(T_base_footprint, 'rgb', 'length', 0.1, 'arrow');
% Initialize Base Homogeneus transformation
if contacts(1) == 0
    T_base0 = transl(0,0,ROBOT(1).base.t(3));
elseif contacts(1) == 1
    T_base0 = transl(0,0,ROBOT_CONTACT(1).fkine(q_contact(1,:)).t(3));
end

% Initialize graphic handlers
for i = 1:N_limb
    h_root0{i} = trplot(eye(4)); 
end
h_base0 = trplot(T_base0,'rgb', 'length', 0.1, 'arrow');
h_patch0 = patch([0 0 0 0], [0 0 0 0], 'b');
% Plot other useful frames (limb root, base) and base patch
[T_limb_root, r_base, h_root, h_base, h_patch] = update_frames_contact(ROBOT, q, T_base0, W,L, h_root0, h_base0, h_patch0);
title('Robot Swing & Contact configuration');

pause % Wait for user signal 
%% Experiment with Base motion
% 1) Translate the base
% 2) Plot the new limbs configuration
% 3) Move the useful frames and base patch
[q1,T_base] = translate_base(ROBOT,T_base0, q, 0.05, 0.05, 0.0);
plot_robot(ROBOT, q1);
[T_limb_root, r_base, h_root, h_base, h_patch] = update_frames_contact(ROBOT, q1, T_base, W,L, h_root, h_base, h_patch);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% USEFUL FUNCTIONS IMPLEMENTATION %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% INITIALIZE ROBOT 
% This function return the Robot model as an array of SerialLink objects,
% each one as a limb. 
% INPUT: 
% - limb         = SerialLink object to use as limb in swing mode
% - limb_contact = SerialLink object to use as limb in contact mode
% - contact_mask = logic array indicating which limb is in contact and
%                  which one not
% OUTPUT:
% - Robot        = Robot model 
function Robot = Robot_model(limb, limb_contact, contact_mask, tool_length)
    N_limb = length(contact_mask);
    g = [0; 0; 9.81];             % Gravity vector
    W = 0.35; % Base width
    L = 0.35; % Base Length

    % Homogeneus Transformations for Base to Limbroot position CONTACT MODE
    q0_contact = zeros(1,limb.n); % zero joint configuration 
    tx_ee_contact = limb_contact.fkine(q0_contact).t(1); % x-coordinate of limb tool = limb length
    t = tx_ee_contact*sqrt(2)/2; % transaltion to move limb base to correct position
    T_LF_contact = transl(L/2+t, W/2+t, tool_length)*trotz(-(pi/2+pi/4)*180/pi)*limb_contact.base.T;
    T_LH_contact =  transl(-L/2-t, W/2+t, tool_length)*trotz(-(pi/4)*180/pi)*limb_contact.base.T;
    T_RH_contact =  transl(-L/2-t, -W/2-t, tool_length)*trotz((pi/4)*180/pi)*limb_contact.base.T;
    T_RF_contact =  transl(L/2+t, -W/2-t, tool_length)*trotz((pi/4+pi/2)*180/pi)*limb_contact.base.T;
    T_contact(:,:,1) = T_LF_contact;
    T_contact(:,:,2) = T_LH_contact;
    T_contact(:,:,3) = T_RH_contact;
    T_contact(:,:,4) = T_RF_contact;  
    T_tool = trotz(pi*180/pi); % rotate the tool frame as in the URDF


    % Homogeneus Transformations for Base to Limbroot position SWING MODE
    q0_contact_swing = [0,0,pi/2,0,0,0,0];
    tz_ee = limb.fkine(q0_contact_swing).t(3); % z-coordinate of limb tool = limb height
    T_LF = transl(L/2, W/2, -tz_ee)*trotz(pi/4*180/pi);
    T_LH =  transl(-L/2, W/2, -tz_ee)*trotz((pi/2+pi/4)*180/pi);
    T_RH =  transl(-L/2, -W/2, -tz_ee)*trotz((-pi/2-pi/4)*180/pi);
    T_RF =  transl(L/2, -W/2, -tz_ee)*trotz(-pi/4*180/pi);
    T(:,:,1) = T_LF;
    T(:,:,2) = T_LH;
    T(:,:,3) = T_RH;
    T(:,:,4) = T_RF;
    
    names = ['New_LF'; 'New_LH'; 'New_RH'; 'New_RF'];

    for i=1:N_limb
        if contact_mask(i) == 0
            Robot(i) = SerialLink(limb, 'name', names(i,:), 'gravity', g, 'base', T(:,:,i))
        elseif contact_mask(i) == 1
            Robot(i) = SerialLink(limb_contact, 'name', strcat(names(i,:),'_{contact}'), 'gravity', g, 'base', T_contact(:,:,i), 'tool', T_tool);
        end
    end
end

%% PLOT ROBOT
% This function plot the ROBOT Limbs in contact state with configuration q
% INPUT: 
% - ROBOT = array of SerialLink object, each one describing one limb
% - q     = current limbs configuration, as an [N_limb x N_joint] matrix
function plot_robot(ROBOT, q)
    N_limb = length(ROBOT);
    contacts = check_contact_limbs(ROBOT);
    for i = 1:N_limb
        if contacts(i) == 1
            ROBOT(i).plot(q(i,:),'workspace', [-0.5 0.5 -0.5 0.5 0.0 0.5], 'noshadow', 'notiles', 'scale', 0.6); 
        elseif contacts(i) == 0
            ROBOT(i).plot(q(i,:),'workspace', [-0.5 0.5 -0.5 0.5 0.0 0.5], 'noshadow','nobase', 'notiles', 'scale', 0.6); 
        end
    end
    
    % Set axis limits manually to ensure the entire robot is visible
    xlim([-0.6 0.6]);  % Set x-axis limits
    ylim([-0.6 0.6]);  % Set y-axis limits
    zlim([ 0.0 0.6]);  % Set z-axis limits
    
    % Set equal aspect ratio to avoid distortion
    axis equal; 
    title('Robot zero configuration, Contact limbs');
end

%% CHECK CONTACT LIMBS
% This function return a "mask array" with the limb in contact 
% INPUT: 
% - ROBOT = array of SerialLink object, each one describing one limb
% OTUPUT:
% - contact_mask = logical row vector of N_limb length, which has 1 in
%                  position of limbs in contact state, 0 otherwise
function contact_mask = check_contact_limbs(ROBOT)
    N_limb = length(ROBOT);
    contact_mask = zeros(1,N_limb);
    for i=1:N_limb
        if contains(ROBOT(i).name, 'contact')
            contact_mask(i) = 1; % this limb is in contact
        end
    end  
end

%% FRAMES UPDATE
% This function updates the visualization of limb_root and base frames
% INPUT: 
% - ROBOT      = array of SerialLink object, each one describing one limb
% - q          = current limbs configuration, as an [N_limb x N_joint] matrix
% - T_base     =  4x4 Homomgeneus Transformation, base wrt fixed frame
% - W          = base width in meters
% - L          = base length in meters
% - h_root_in  = handle to root_link SE3 plot graphic, as [1x4] cell, each limb
% - h_base_in  = handle to base SE3 graphic
% - h_patch_in = handle to base patch graphic
% OUTPUT: 
% - T_limb_root = 4x4 Homomgeneus Transformation, limb root wrt fixed frame
% - r_base      = matrix [N_limb x 3] containing in each row the vector of the limb
%                 "graspoing point" with the robot base (limb-base attachment)
% - h_root      = New handle to root_link SE3 plot graphic, as [1x4] cell, each limb
% - h_base      = New handle to base SE3 graphic
% - h_patch     = New handle to base patch graphic

function [T_limb_root,r_base, h_root, h_base, h_patch] = update_frames_contact(ROBOT, q, T_base, W, L, h_root_in, h_base_in, h_patch_in)
    N_limb = length(ROBOT);
    % delete graphics 
    delete(h_patch_in);
    for i=1:N_limb
        delete(h_root_in{i});
    end
    delete(h_base_in);
    % Initialize homogeneus transformation matrices
    T_coxa_limb_root = zeros(4,4,N_limb);
    T_coxa = zeros(4,4,N_limb);
    T_limb_root = zeros(4,4,N_limb);
    
    % check which limbs are in contact
    contacts = check_contact_limbs(ROBOT);

    for i = 1:N_limb
        if contacts(i) == 1
            T_coxa_limb_root(:,:,i) = trotz(-q(i,7)*180/pi);
            T_coxa(:,:,i) = ROBOT(i).fkine(q(i,:));
            T_limb_root(:,:,i) = T_coxa(:,:,i)*T_coxa_limb_root(:,:,i);
            h_root{i} = trplot(T_limb_root(:,:,i), 'rgb', 'length', 0.08, 'arrow');
            
            T = (transl(-sqrt((W/2)^2+(L/2)^2), 0, 0)*trotz(-pi/4*180/pi))^-1;
            r_base_tilde(i,:) = T_limb_root(:,:,i)*T(:,4);
            r_base(i,:) = r_base_tilde(i,1:3);
        elseif contacts(i) == 0
            T_limb_root(:,:,i) = ROBOT(i).base; % when not in contact, limb root coincide with limb base
            h_root{i} = trplot(T_limb_root(:,:,i), 'rgb', 'length', 0.08, 'arrow');
            % No need to compute root to base radius, because no
            % contribution to the grasp matrix is given by this limb
            T = (transl(-sqrt((W/2)^2+(L/2)^2), 0, 0)*trotz(-pi/4*180/pi))^-1;
            r_base_tilde(i,:) = T_limb_root(:,:,i)*T(:,4);
            r_base(i,:) = r_base_tilde(i,1:3);
        end
    end

    h_base = trplot(T_base, 'rgb', 'length', 0.1, 'arrow', 'framelabel', 'base');
    h_patch = patch('XData',[T_limb_root(1,4,1), T_limb_root(1,4,2), T_limb_root(1,4,3), T_limb_root(1,4,4)],'YData',[T_limb_root(2,4,1), T_limb_root(2,4,2), T_limb_root(2,4,3), T_limb_root(2,4,4)],'ZData',[T_limb_root(3,4,1), T_limb_root(3,4,2), T_limb_root(3,4,3), T_limb_root(3,4,4)], 'FaceColor', 'b', 'FaceAlpha', 0.8);
end

%% COMPUTE BASE TRANSLATION
% This function update the robot configuration and base homogeneus
% transformation, accordingly to the desired base translation 
% INPUT:
% - ROBOT    = array containing each limb as SerialLink object
% - T_base   = 4x4 Homomgeneus Transformation, base wrt fixed frame
% - q        = current limbs configuration as [N_limb x N_joint] matrix
% - x_motion = base translation on x direction, in base coordinates
% - y_motion = base translation on x direction, in base coordinates
% - z_motion = base translation on x direction, in base coordinates
% OUTPUT: 
% - q_new = [N_limb x N_joint] matrix of new limbs configuration to achieve desired translation
% - T_base    = New 4x4 Homomgeneus Transformation, base wrt fixed frame,
%               after translation

function [q_new, T_base] = translate_base(ROBOT,T_base_in,  q, x_motion, y_motion, z_motion)
    N_limb = length(ROBOT);
    % check which limbs are in contact
    contacts = check_contact_limbs(ROBOT);

    t_ee = transl(x_motion, y_motion, z_motion); % define end desired translation
    T_base = T_base_in*t_ee; % update base frame

    for i = 1:N_limb
        if contacts(i) == 1 % only contacts limb move the base
            T_tool_base(:,:,i) = (ROBOT(i).fkine(q(i,:)).T)^-1*T_base;
            R_tool_base(:,:,i) = T_tool_base(1:3,1:3,i);
            t_ee_new = [R_tool_base(:,:,i), zeros(3,1); zeros(1,3), 1]*t_ee;
            T_ee(:,:,i) = ROBOT(i).fkine(q(i,:)).T*transl(t_ee_new(1,4), t_ee_new(2,4), t_ee_new(3,4));
            q_new(i,:) = ROBOT(i).ikine(T_ee(:,:,i));
        elseif contacts(i) == 0 % limbs not in contact move with the base
            t0 = (ROBOT(i).base.R)'*T_base_in(1:3,1:3)*t_ee(1:3,4);
            ROBOT(i).base = ROBOT(i).base.T * transl(t0(1), t0(2), t0(3));
            q_new(i,:) = q(i,:);
        end
    end
end

%% COMPUTE GRASP MATRIX 
% this function compute the grasp matrix given the vector of the limbr root
% frames, in the fixed inertia frame

% INPUT: 
% - r = matrix [N_limb_contact x 3] containing in each row the vector of
%       the limb incontact "graspoing point" with the robot base
% OUTPUT: 
% - grasp_matrix = [6 x (N_limb_contact*N_joint)] matrix, representing grasp
%                  relation
function grasp_matrix = compute_grasp_matrix(r)
    grasp_matrix = [];
    for i=1:length(r)
        R(:,:,i) = [0 -r(i,3) r(i,2); r(i,3) 0 -r(i,1); -r(i,2) r(i,1) 0];
        W(:,:,i) = [eye(3), zeros(3,3); R(:,:,i), eye(3)];
        grasp_matrix = [grasp_matrix, W(:,:,i)];
    end
end

%% COMPUTE BASE MANIPULABILITY CORE
% This function compute the core 6x6 matrix of the base velocity elliposid

% INPUT:
% - ROBOT = array of SerialLink, each limb
% - q     = current robot configuration, N_limb*6 matrix (each row
%           correspond to one limb configuration)
% - W     = grasp matrix
% OUTPUT: 
% - base_ellispoid = correspond to the core matrix E s.t the ellipsoid is
%                    descriped  by v*(E)^-1*v'<=1
% -Ja              = corresponding grasp Jacobian matrix, relating base absolute speed
%                    with joint speed

% [TODO]: Fix this function to compute it based on the amount of limb in
% contact, extracting this information from the dimension of W or something
% else..

function [base_ellipsoid,Ja] = compute_base_ellipsoid(ROBOT, q, W)
    N_limb = length(ROBOT); 
    N_joint = ROBOT(1).n;
    J_full = zeros(N_limb*6,N_limb*N_joint);
    for i=1:N_limb
        J_full(1+(i-1)*6:6+(i-1)*6, 1+(i-1)*N_joint:N_joint+(i-1)*N_joint) = ROBOT(i).jacobe(q);
    end
    Ja = (J_full'*pinv(W))';
    base_ellipsoid = Ja*Ja';

end





