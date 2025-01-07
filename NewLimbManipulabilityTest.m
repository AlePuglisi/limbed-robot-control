clc
clear 
close all 

% In this script I want to test the concept of base manipulability for a
% simpler robotic leg structure
%% TEST BASE MANIPULABILITY THEORY %%

%% DEFINE NEW LIMB SWING/CONTACT MODEL %%
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
Limb_swing = SerialLink(L);
Limb_swing.name = 'Limb';
Limb_swing.gravity = [0; 0; 9.81]; %gravity acceleration vector expressed in the base frame 

qz = zeros(1,Limb_swing.n);

figure('Name', 'Limb LF, SWINGING DH (Coxa-EndEffector)');
%limbero.plot3d([0, pi/2, 0, 0, 0, 0, 0], 'workspace', [-1 1 -1 1 -1 1], 'view', [30 30], 'scale', 0.6);
Limb_swing.plot(qz, 'workspace', [-1 1 -1 1 -1 1], 'view', [30 30], 'scale', 0.6,'jvec', 'nobase', 'noshadow', 'notiles');
title('Limb LF DH, swinging (q = qz)');

% CONTACT
a_contact = [a2, 0, 0];
d_contact = [0, 0, -d1];
alpha_contact = [0, pi/2, 0];
offset_contact = [0, 0, pi];

% Generate the link object associated with the robot model (DH param); 
L1_contact = Link('d', d_contact(1), 'a', a_contact(1), 'alpha', alpha_contact(1), 'offset', offset_contact(1)); % coxa
L2_contact = Link('d', d_contact(2), 'a', a_contact(2), 'alpha', alpha_contact(2), 'offset', offset_contact(2)); % femur
L3_contact = Link('d', d_contact(3), 'a', a_contact(3), 'alpha', alpha_contact(3), 'offset', offset_contact(3)); %tibia

L_contact = [L1_contact, L2_contact, L3_contact];
Limb_contact = SerialLink(L_contact);
Limb_contact.name = 'Limb_{contact}';
Limb_contact.gravity = [0; 0; 9.81]; %gravity acceleration vector expressed in the base frame 
Limb_contact.base = trotx(-pi/2*180/pi)*transl(0,-a3,0);

figure('Name', 'Limb LF, CONTACT DH (EndEffector-Coxa)');
%limbero.plot3d([0, pi/2, 0, 0, 0, 0, 0], 'workspace', [-1 1 -1 1 -1 1], 'view', [30 30], 'scale', 0.6);
Limb_contact.plot(qz, 'workspace', [-1 1 -1 1 0 1], 'view', [30 30], 'scale', 0.6,'jvec', 'noshadow', 'notiles');
title('Limb LF DH, Contact (q = qz)');

%% DEFINE NEW ROBOT %%
ROBOT = Robot_model(Limb_swing, Limb_contact, [1 1 1 1], 0);
for i=1:length(ROBOT)
    q(i,:) = [0,0, 0];
end

figure('Name', 'ROBOT')
hold on 

plot_robot(ROBOT, q);

N_link = ROBOT(1).n;
N_limb = length(ROBOT);

q0_swing = [0,pi/2, 0];
q0_contact = zeros(1,N_link);
contacts = check_contact_limbs(ROBOT);
for i=1:N_limb
    if contacts(i) == 1
        q(i,:) = q0_contact;
    elseif contacts(i) == 0
        q(i,:) = q0_swing;

    end
end


% I take as inertial frame the base-footprint center in the zero configuration
T_base_footprint = eye(4,4);
trplot(T_base_footprint, 'rgb', 'length', 0.2, 'arrow');

% Represent Base Frame and Shape (as Rectangular patch) 
% Initialize Base Homogeneus transformation
if contacts(1) == 0
    T_base0 = transl(0,0,Limb_swing.base.t(3));
elseif contacts(1) == 1
    T_base0 = transl(0,0,Limb_contact.fkine(q0_contact).t(3));
end

% Initialize graphic handlers
for i = 1:N_limb
    h_root0{i} = trplot(eye(4)); 
end
h_base0 = trplot(T_base0,'rgb', 'length', 0.2, 'arrow');
h_CoM0 = plot3(0,0,0);
h_patch0 = patch([0 0 0 0], [0 0 0 0], 'b');
h_support0 = patch([0 0 0 0], [0 0 0 0], 'g');

% Plot other useful frames (limb root, base) and base patch
W = 0.45; % Base width
L = 0.45; % Base Length
[T_limb_root, r_base, h_root, h_base, h_patch, h_support, h_CoM] = update_frames(ROBOT, q, T_base0, W,L, h_root0, h_base0, h_patch0, h_support0, h_CoM0);
title('Robot Swing & Contact configuration');

%% Experiment with Base manipulability Ellipsoid 
% Compute Grasp matrix and then Ellipsoid core
grasp_matrix = compute_grasp_matrix(r_base);
[E_base, Ja] = compute_base_ellipsoid(ROBOT, q, grasp_matrix);
% Plot ellipsoid, in the base frame
h_ellipse = plot_ellipse(E_base(1:3,1:3),[T_base0(1,4), T_base0(2,4), T_base0(3,4)], 'r', 'alpha', 0.6);

pause % Wait for user signal 
%% Experiment with Base motion
% 1) Translate the base
% 2) Plot the new limbs configuration
% 3) Move the useful frames and base patch
[q1,T_base] = translate_base(ROBOT,T_base0, q, 0.01, 0.0, 0.0);
plot_robot(ROBOT, q);
[T_limb_root, r_base, h_root, h_base, h_patch, h_support, h_CoM] = update_frames(ROBOT, q1, T_base, W,L, h_root, h_base, h_patch, h_support, h_CoM);

% Update Base Ellipsoid
% Compute Grasp matrix and then Ellipsoid core
grasp_matrix = compute_grasp_matrix(r_base);
[E_base, Ja] = compute_base_ellipsoid(ROBOT, q1, grasp_matrix);
% Plot ellipsoid, in the base frame, removing the previous one
delete(h_ellipse);
h_ellipse = plot_ellipse(E_base(1:3,1:3),[T_base(1,4), T_base(2,4), T_base(3,4)], 'r', 'alpha', 0.6);

%% FUNCTIONS 

%% Define robot model
function Robot = Robot_model(limb, limb_contact, contact_mask, tool_length)
    N_limb = length(contact_mask);
    g = [0; 0; 9.81];             % Gravity vector
    W = 0.45; % Base width
    L = 0.45; % Base Length

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

    % Homogeneus Transformations for Base to Limbroot position SWING MODE
    q0_contact_swing = [0,pi/2,0];
    tz_ee = limb.fkine(q0_contact_swing).t(3); % z-coordinate of limb tool = limb height
    T_LF = transl(L/2, W/2, -tz_ee)*trotz(pi/4*180/pi);
    T_LH =  transl(-L/2, W/2, -tz_ee)*trotz((pi/2+pi/4)*180/pi);
    T_RH =  transl(-L/2, -W/2, -tz_ee)*trotz((-pi/2-pi/4)*180/pi);
    T_RF =  transl(L/2, -W/2, -tz_ee)*trotz(-pi/4*180/pi);
    T(:,:,1) = T_LF;
    T(:,:,2) = T_LH;
    T(:,:,3) = T_RH;
    T(:,:,4) = T_RF;
    
    names = ['LF*'; 'LH*'; 'RH*'; 'RF*'];

    for i=1:N_limb
        if contact_mask(i) == 0
            Robot(i) = SerialLink(limb, 'name', names(i,:), 'gravity', g, 'base', T(:,:,i));
        elseif contact_mask(i) == 1
            Robot(i) = SerialLink(limb_contact, 'name', strcat(names(i,:),'_{contact}'), 'gravity', g, 'base', T_contact(:,:,i));
        end
    end
end

%% Plot the robot
function plot_robot(ROBOT, q)
    N_limb = length(ROBOT);
    contacts = check_contact_limbs(ROBOT);
    for i = 1:N_limb
        if contacts(i) == 1
            ROBOT(i).plot(q(i,:),'workspace', [-0.8 0.8 -0.8 0.8 -0.0 1.0], 'noshadow', 'notiles', 'scale', 0.6); 
        elseif contacts(i) == 0
            ROBOT(i).plot(q(i,:),'workspace', [-0.8 0.8 -0.8 0.8 -0.0 1.0], 'noshadow','nobase', 'notiles', 'scale', 0.5); 
        end
    end
    
    % Set axis limits manually to ensure the entire robot is visible
    xlim([-0.8 0.8]);  % Set x-axis limits
    ylim([-0.8 0.8]);  % Set y-axis limits
    zlim([ -0.0 1.0]);  % Set z-axis limits
    
    % Set equal aspect ratio to avoid distortion
    %axis equal; 
    title('Robot');
end

%% check limbs in contact
function contact_mask = check_contact_limbs(ROBOT)
    N_limb = length(ROBOT);
    contact_mask = zeros(1,N_limb);
    for i=1:N_limb
        if contains(ROBOT(i).name, 'contact')
            contact_mask(i) = 1; % this limb is in contact
        end
    end  
end

%% Translate the base
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
            q_new(i,:) = ROBOT(i).ikine(T_ee(:,:,i), q(i,:), 'mask', [1 1 1 0 0 0]);
        elseif contacts(i) == 0 % limbs not in contact move with the base
            t0 = (ROBOT(i).base.R)'*T_base_in(1:3,1:3)*t_ee(1:3,4);
            ROBOT(i).base = ROBOT(i).base.T * transl(t0(1), t0(2), t0(3));
            q_new(i,:) = q(i,:);
        end
    end
end

%% Update frames 
function [T_limb_root,r_base, h_root, h_base, h_patch, h_support, h_CoM] = update_frames(ROBOT, q, T_base, W, L, h_root_in, h_base_in, h_patch_in, h_support_in, h_CoM_in)
    N_limb = length(ROBOT);
    % delete graphics 
    delete(h_patch_in);
    delete(h_support_in);
    for i=1:N_limb
        delete(h_root_in{i});
    end
    delete(h_base_in);
    delete(h_CoM_in)
    % Initialize homogeneus transformation matrices
    T_coxa_limb_root = zeros(4,4,N_limb);
    T_coxa = zeros(4,4,N_limb);
    T_limb_root = zeros(4,4,N_limb);
    
    % check which limbs are in contact
    contacts = check_contact_limbs(ROBOT);
    N_limb_contact = sum(contacts);
    t_support = [];
    
    for i = 1:N_limb
        if contacts(i) == 1
            T_coxa_limb_root(:,:,i) = trotz(-q(i,3)*180/pi);
            T_coxa(:,:,i) = ROBOT(i).fkine(q(i,:));
            T_limb_root(:,:,i) = T_coxa(:,:,i)*T_coxa_limb_root(:,:,i);
            h_root{i} = trplot(T_limb_root(:,:,i), 'rgb', 'length', 0.1, 'arrow');
            
            T = (transl(-sqrt((W/2)^2+(L/2)^2), 0, 0)*trotz(-pi/4*180/pi))^-1;
            r_base_tilde(i,:) = T_limb_root(:,:,i)*T(:,4);
            r_base(i,:) = r_base_tilde(i,1:3);
            t_support = [t_support; (ROBOT(i).base.t)'];

        elseif contacts(i) == 0
            T_limb_root(:,:,i) = ROBOT(i).base; % when not in contact, limb root coincide with limb base
            h_root{i} = trplot(T_limb_root(:,:,i), 'rgb', 'length', 0.08, 'arrow');
            % No need to compute root to base radius, because no
            % contribution to the grasp matrix is given by this limb
            r_base(i,:) = zeros(1,3);
        end
    end
    
    h_base = trplot(T_base, 'rgb', 'length', 0.1, 'arrow', 'framelabel', 'base');
    h_patch = patch('XData',[T_limb_root(1,4,1), T_limb_root(1,4,2), T_limb_root(1,4,3), T_limb_root(1,4,4)],'YData',[T_limb_root(2,4,1), T_limb_root(2,4,2), T_limb_root(2,4,3), T_limb_root(2,4,4)],'ZData',[T_limb_root(3,4,1), T_limb_root(3,4,2), T_limb_root(3,4,3), T_limb_root(3,4,4)], 'FaceColor', 'b', 'FaceAlpha', 0.6);
    % Plot support polygon
    tool_length = 0.0; 
    if N_limb_contact ~=4 & N_limb_contact ~=0
        tool_length = tool_length + 0.02; 
    end
    if N_limb_contact ~= 0
        %t_support(:,3) = t_support(:,3) - tool_length;
        h_support = patch('XData',[t_support(:,1)],'YData',[t_support(:,2)],'ZData',[zeros(1,4)], 'FaceColor', 'g', 'FaceAlpha', 0.3);
    else
        h_support = h_support_in;
    end
    % Plot CoM Projection
    x_com = T_base(1,4);
    y_com = T_base(2,4);
    z_com = 0;
    h_CoM = plot3(x_com, y_com, z_com,'o','Color','#FF6E00','MarkerSize',10,'MarkerFaceColor','#FF6E00');
end

%% compute grasp matrix 
function grasp_matrix = compute_grasp_matrix(r)
    grasp_matrix = [];
    r_new = [];
    for i=1:length(r)
        if r(i,1) == 0 & r(i,2) == 0  & r(i,3) == 0     
            % don't save it
        else
            r_new = [r_new; r(i,:)];
        end
    end
    for i=1:length(r_new)
        R(:,:,i) = [0 -r(i,3) r(i,2); r(i,3) 0 -r(i,1); -r(i,2) r(i,1) 0];
        W(:,:,i) = [eye(3), zeros(3,3); R(:,:,i), eye(3)];
        grasp_matrix = [grasp_matrix, W(:,:,i)];
    end
end

%% Base manipulability core
function [base_ellipsoid,Ja] = compute_base_ellipsoid(ROBOT, q, W)
    N_limb = length(ROBOT); 
    N_joint = ROBOT(1).n;
    contacts = check_contact_limbs(ROBOT);
    N_limb_contact = sum(contacts);

    J_full = zeros(N_limb_contact*6,N_limb_contact*N_joint);
    ROBOT_CONTACT = [];
    q_new = [];
    for i=1:N_limb
        if contacts(i) == 1
            ROBOT_CONTACT = [ROBOT_CONTACT, ROBOT(i)];
            q_new = [q_new; q(i,:)];
        end
    end
    
    for i=1:N_limb_contact
        % A change in the reference frame is needed, we use common origin jacobian  
        % J_full(1+(i-1)*6:6+(i-1)*6, 1+(i-1)*N_joint:N_joint+(i-1)*N_joint) = tr2jac(ROBOT_CONTACT(i).base, 'samebody')*ROBOT_CONTACT(i).jacob0(q_new(i,:));
        J_full(1+(i-1)*6:6+(i-1)*6, 1+(i-1)*N_joint:N_joint+(i-1)*N_joint) = ROBOT_CONTACT(i).jacob0(q_new(i,:));
    end
    Ja = (J_full'*pinv(W))';
    base_ellipsoid = Ja*Ja';

end
