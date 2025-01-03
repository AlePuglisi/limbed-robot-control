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

q_contact = [0,0,pi/2,0,0,0,0];
%% SWING LEGS MODEL %%
% Homogeneus Transformations for Base to Limbroot position
tz_ee = limbero.fkine(q_contact).t(3); % z-coordinate of limb tool = limb height

T_LF = transl(-L/2, W/2, -tz_ee)*trotz((pi/2+pi/4)*180/pi);
T_LH =  transl(-L/2, -W/2, -tz_ee)*trotz((pi+pi/4)*180/pi);
T_RH =  transl(L/2, -W/2, -tz_ee)*trotz((-pi/4)*180/pi);
T_RF =  transl(L/2, W/2, -tz_ee)*trotz((pi/4)*180/pi);
% Leg redefinition
LF_leg = SerialLink(limbero, 'name', 'LF', 'gravity', g, 'base', T_LF);
LH_leg = SerialLink(limbero, 'name', 'LH', 'gravity', g, 'base', T_LH);
RH_leg = SerialLink(limbero, 'name', 'RH', 'gravity', g, 'base', T_RH);
RF_leg = SerialLink(limbero, 'name', 'RF', 'gravity', g, 'base', T_RF);

ROBOT = [LF_leg, LH_leg, RH_leg, RF_leg]; 

q0 = zeros(1,N_link);
q1 = [0,0,pi/2,0,0,0,0];
q = q1; 

% Plot the robot 
figure('Name', 'LIMBERO+GRIEEL q=qz')
hold on
for i = 1:N_limb
    ROBOT(i).plot(q,'workspace', [-0.5 0.5 -0.5 0.5 -0.5 0.5], 'nobase', 'noshadow', 'notiles', 'view', [30 30], 'scale', 0.6); 
end

% Set axis limits manually to ensure the entire robot is visible
xlim([-1 1]);  % Set x-axis limits
ylim([-1 1]);  % Set y-axis limits
zlim([-1 1]);  % Set z-axis limits

% Set equal aspect ratio to avoid distortion
axis equal; 

% Add fictitious base, for now fixed 
% TODO: make the base updated with the limb root position
patch('XData',[-L/2,-L/2,L/2,L/2],'YData',[W/2 -W/2 -W/2 W/2],'ZData',[-tz_ee -tz_ee -tz_ee -tz_ee], 'FaceColor', 'b', 'FaceAlpha', 0.8);

hold off

title('Robot Contact configuration, Swing limbs');

%% CONTACT LEGS MODEL %% 
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

q0_contact = zeros(1,N_link);
for i = 1:N_limb
    q_contact(i,:) = q0_contact; % collect in a N_limbxN_joint matrix
end

% Plot the robot 
figure('Name', 'LIMBERO+GRIEEL CONTACT q=qz')
hold on
for i = 1:N_limb
    ROBOT_CONTACT(i).plot(q_contact(i,:),'workspace', [-0.5 0.5 -0.5 0.5 0.0 0.5], 'noshadow', 'notiles', 'scale', 0.6); 
end

% Set axis limits manually to ensure the entire robot is visible
xlim([-0.6 0.6]);  % Set x-axis limits
ylim([-0.6 0.6]);  % Set y-axis limits
zlim([ 0.0 0.6]);  % Set z-axis limits

% Set equal aspect ratio to avoid distortion
axis equal; 
title('Robot zero configuration, Contact limbs');

% Represent Base Frame and Shape (as Rectangular patch) 
% I take as inertial frame the base-footprint center in the zero configuration
T_base_footprint = eye(4,4);
trplot(T_base_footprint, 'rgb', 'length', 0.1, 'arrow');

T_coxa_limb_root = zeros(4,4,N_limb);
T_coxa = zeros(4,4,N_limb);
T_limb_root = zeros(4,4,N_limb);

limb_names = ['LF'; 'LH'; 'RH'; 'RF'];

for i = 1:N_limb
    T_coxa_limb_root(:,:,i) = trotz(-q_contact(i,7)*180/pi);
    T_coxa(:,:,i) = ROBOT_CONTACT(i).fkine(q_contact(i,:));
    T_limb_root(:,:,i) = T_coxa(:,:,i)*T_coxa_limb_root(:,:,i);
    trplot(T_limb_root(:,:,i), 'rgb', 'length', 0.08, 'arrow');
end

T_base = T_limb_root(:,:,1)*transl(-sqrt((W/2)^2+(L/2)^2), 0, 0)*trotz(-pi/4*180/pi);
trplot(T_base, 'rgb', 'length', 0.1, 'arrow');
base_x = T_base(1,4);
base_y = T_base(2,4);
base_z = T_base(3,4);

patch('XData',[base_x - L/2, base_x - L/2, base_x + L/2, base_x + L/2],'YData',[base_y + W/2, base_y - W/2, base_y - W/2, base_y + W/2],'ZData',[base_z base_z base_z base_z], 'FaceColor', 'b', 'FaceAlpha', 0.8);


