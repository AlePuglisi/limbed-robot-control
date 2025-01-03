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

%% SWING LEGS MODEL %%
% Homogeneus Transformations for Base to Limbroot position
T_LF = transl(-L/2, W/2, 0)*trotz((pi/2+pi/4)*180/pi);
T_LH =  transl(-L/2, -W/2, 0)*trotz((pi+pi/4)*180/pi);
T_RH =  transl(L/2, -W/2, 0)*trotz((-pi/4)*180/pi);
T_RF =  transl(L/2, W/2, 0)*trotz((pi/4)*180/pi);
% Leg redefinition
LF_leg = SerialLink(limbero, 'name', 'LF', 'gravity', g, 'base', T_LF);
LH_leg = SerialLink(limbero, 'name', 'LH', 'gravity', g, 'base', T_LH);
RH_leg = SerialLink(limbero, 'name', 'RH', 'gravity', g, 'base', T_RH);
RF_leg = SerialLink(limbero, 'name', 'RF', 'gravity', g, 'base', T_RF);

ROBOT = [LF_leg, LH_leg, RH_leg, RF_leg]; 

q0 = zeros(1,N_link);
q = q0; 

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
patch([-L/2,-L/2,L/2,L/2], [W/2 -W/2 -W/2 W/2], [0 0 0 0], 'FaceColor', 'b', 'FaceAlpha', 0.8);

hold off

title('Robot zero configuration, Swing limbs');

%% CONTACT LEGS MODEL %% 
g = [0; 0; 9.81]; % Gravity vector
q0_contact = zeros(1,N_link);
% Homogeneus Transformations for Base to Limbroot position
tx_ee = limbero_contact.fkine(q0_contact).t(1);
t = tx_ee*sqrt(2)/2;
T_LF = transl(L/2+t, W/2+t, 0)*trotz(-(pi/2+pi/4)*180/pi)*limbero_contact.base.T;
T_LH =  transl(-L/2-t, W/2+t, 0)*trotz(-(pi/4)*180/pi)*limbero_contact.base.T;
T_RH =  transl(-L/2-t, -W/2-t, 0)*trotz((pi/4)*180/pi)*limbero_contact.base.T;
T_RF =  transl(L/2+t, -W/2-t, 0)*trotz((pi/4+pi/2)*180/pi)*limbero_contact.base.T;
% Leg redefinition
LF_leg_contact = SerialLink(limbero_contact, 'name', 'LF_{contact}', 'gravity', g, 'base', T_LF);
LH_leg_contact = SerialLink(limbero_contact, 'name', 'LH_{contact}', 'gravity', g, 'base', T_LH);
RH_leg_contact = SerialLink(limbero_contact, 'name', 'RH_{contact}', 'gravity', g, 'base', T_RH);
RF_leg_contact = SerialLink(limbero_contact, 'name', 'RF_{contact}', 'gravity', g, 'base', T_RF);

ROBOT_CONTACT = [LF_leg_contact, LH_leg_contact, RH_leg_contact, RF_leg_contact]; 

q0_contact = zeros(1,N_link);
q_contact = q0_contact;

% Plote the robot 
figure('Name', 'LIMBERO+GRIEEL CONTACT q=qz')
hold on
for i = 1:N_limb
    ROBOT_CONTACT(i).plot(q_contact, 'nobase', 'noshadow', 'notiles'); 
end

