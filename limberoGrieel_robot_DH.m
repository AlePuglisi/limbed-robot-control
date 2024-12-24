clc 
clear 
close all 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% LIMBERO + GRIEEL DH ROBOT MODEL %%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% run this code to create thr robot model represented by 4 limbs 
% This is a Test code..

%%%%%%%%%%%%%%%%%%%%%%%
%% DH MODEL CREATION %%
%%%%%%%%%%%%%%%%%%%%%%%
%%%%  SINGLE LIMB  %%%%

%% URDF MODEL 
% limbero leg model in URDF 
% created as RigidBodyTree
limbero_urdf = importrobot("urdf/LEG_LFGRIPPER.urdf", 'MeshPath', 'lbr_description/meshes');
limbero_urdf.Gravity = [0 0 -9.81]; % set up gravity 

limb_dof = 7; % constant dof 

limbero_urdf.DataFormat = 'row'; % representaion data format

% initialize configuartion vector
config0 = homeConfiguration(limbero_urdf); 
% change configuration:
% config(B2C_index) = pi/4; 
% config(C2F_index) = -pi/6; 
% config(F2T_index) = pi/2; 

% show model in default configuration
figure('Name', 'Limbero URDF')
show(limbero_urdf, config0); 
title('Limbero+Grieel LF leg URDF (RigidBodyTree) (q = config0)');

% DYNAMIC ANALYSIS:
% initialize joint space velocity for dynamics 
% q_dot = zeros(1,limb_dof); 
% q_dot(F2T_index) = pi/10; 

% dynamic tensors computation in current configuration 
% M = massMatrix(limbero_urdf, config0);
% C = velocityProduct(limbero_urdf, config0, q_dot)';
% g = gravityTorque(limbero_urdf, config0)';
% tau = zeros(7,1);

%% DH MODEL 
% KINEMATIC PARAMETERS
% DH parameters LIMBERO+GRIEEL

N_link = 7; 

% initial offset taken from URDF
F2T_init_offset = 1.364075222;
T2E_init_offset = 0.2067211047;

% derive DH parameters from URDF frame relationship between links 
T_coxa2femur = limbero_urdf.getTransform(config0, 'LF_coxa')\limbero_urdf.getTransform(config0, 'LF_femur');
T_femur2tibia = limbero_urdf.getTransform(config0, 'LF_femur')\limbero_urdf.getTransform(config0, 'LF_tibia');
T_tibia2end = limbero_urdf.getTransform(config0, 'LF_tibia')\limbero_urdf.getTransform(config0, 'LF_limb_end');
T_end2driving = limbero_urdf.getTransform(config0, 'LF_limb_end')\limbero_urdf.getTransform(config0, 'LF_driving_Link');
T_driving2gripper = limbero_urdf.getTransform(config0, 'LF_driving_Link')\limbero_urdf.getTransform(config0, 'LF_gripper_Link');
a1 = T_coxa2femur(1,4);
a2 = T_femur2tibia(1,4);
a3 = T_tibia2end(1,4);
d5 = T_end2driving(1,4);
a5 = T_end2driving(3,4);
a6 = T_driving2gripper(3,4);
d7 = T_driving2gripper(1,4) + 0.1; % 0.1 is the length of the gripper itslef 

% define vector of DH parameters 
alpha = [-pi/2,0,0,-pi/2,pi/2, -pi/2, 0]; %[in rad]
a = [a1,a2,a3,0,a5,a6, 0]; %[in m]
d = [0,0,0,0,d5,0,d7]; %[in m]

%generate the link object associated with the robot model (DH param); 
L1 = Link('d', d(1), 'a', a(1), 'alpha', alpha(1)); % coxa
L2 = Link('d', d(2), 'a', a(2), 'alpha', alpha(2)); % femur
L3 = Link('d', d(3), 'a', a(3), 'alpha', alpha(3)); %tibia
L4 = Link('d', d(4), 'a', a(4), 'alpha', alpha(4), 'offset', -pi/2); % foot_end
L5 = Link('d', d(5), 'a', a(5), 'alpha', alpha(5)); % wristH
L6 = Link('d', d(6), 'a', a(6), 'alpha', alpha(6)); % wrist V
L7 = Link('d', d(7), 'a', a(7), 'alpha', alpha(7)); % driving 
% assign name to each link 
L1.name = 'coxa'; 
l2.name = 'femur'; 
L3.name = 'tibia';
L4.name = 'limb_end'; 
L5.name = 'wristH';
L6.name = 'wristV';
L7.name = 'driving';
L= [L1 L2 L3 L4 L5 L6 L7];

%% Create the robot
limbero = SerialLink(L);
limbero.name = 'LIMBERO';
limbero.gravity = [0;0;9.81]; %gravity acceleration vector expressed in the base frame 
qz = zeros(1,limbero.n);

%% CONVERSION OF DYNAMIC PROPERTY FROM URDF TO DH 
% coxa
T_dh_coxa = limbero.A(1, qz);
T_urdf_coxa = limbero_urdf.getTransform(config0, 'LF_coxa');
T_urdf2dh_coxa = T_dh_coxa.T\T_urdf_coxa;
% femur
T_dh_femur = limbero.A(1:2, qz);
T_urdf_femur = limbero_urdf.getTransform(config0, 'LF_femur');
T_urdf2dh_femur = T_dh_femur.T\T_urdf_femur;
% tibia
T_dh_tibia = limbero.A(1:3, qz);
T_urdf_tibia = limbero_urdf.getTransform(config0, 'LF_tibia');
T_urdf2dh_tibia = T_dh_tibia.T\T_urdf_tibia;
% limb_end
T_dh_limb_end = limbero.A(1:4, qz);
T_urdf_limb_end = limbero_urdf.getTransform(config0, 'LF_limb_end');
T_urdf2dh_limb_end = T_dh_limb_end.T\T_urdf_limb_end;
% wristH_Link
T_dh_wristH = limbero.A(1:5, qz);
T_urdf_wristH = limbero_urdf.getTransform(config0, 'LF_wristV_Link');
T_urdf2dh_wristH = T_dh_wristH.T\T_urdf_wristH;
% wristV_Link
T_dh_wristV = limbero.A(1:6, qz);
T_urdf_wristV = limbero_urdf.getTransform(config0, 'LF_driving_Link');
T_urdf2dh_wristV = T_dh_wristV.T\T_urdf_wristV;
% driving_Link
T_dh_driving = limbero.A(1:7, qz);
T_urdf_driving = limbero_urdf.getTransform(config0, 'LF_gripper_Link');
T_urdf2dh_driving = T_dh_driving.T\T_urdf_driving;

% define a (4x4)x7 matrix of frame transformation 
T_urdf2dh = zeros(4, 4, limbero.n);
T_urdf2dh(:,:,1) = T_urdf2dh_coxa; 
T_urdf2dh(:,:,2) = T_urdf2dh_femur; 
T_urdf2dh(:,:,3) = T_urdf2dh_tibia; 
T_urdf2dh(:,:,4) = T_urdf2dh_limb_end; 
T_urdf2dh(:,:,5) = T_urdf2dh_wristH; 
T_urdf2dh(:,:,6) = T_urdf2dh_wristV; 
T_urdf2dh(:,:,7) = T_urdf2dh_driving; 

% REDUCTION ratio n, from ROBOTIS spec.
% B2C, C2F: XM540-W270-R [n = 272.5] https://emanual.robotis.com/docs/en/dxl/x/xm540-w270/
% F2T, T2G, driving: XM430-W350-R [ n = 353.5] https://emanual.robotis.com/docs/en/dxl/x/xm430-w350/
% wristH, wristV: 2XL430-W250-T [n = 257.4] https://emanual.robotis.com/docs/en/dxl/x/2xl430-w250/
% (locking: XL430-W250-T, NOT USED IN THE MODEL)
% REDUCTION RATIO 
n = [272.5, 272.5, 353.5, 353.5, 257.4, 257.4, 353.5]; 

% MOTOR INERTIA (around rotation axis) 
% from ROBOTIS inertia specification of the motors
Jm = [6.0843342e+04, 6.0843342e+04, 2.1199469e+04, 2.1199469e+04, 2.6995309e+04,2.6995309e+04, 2.1199469e+04];% [g*mm^2] 
% in [kg*m^2], e-09 as conversion factor
Jm = Jm*1e-09; %[kg*m^2]

% extract URDF dynamic properties and convert into DH frames 
for i=1:limbero.n
    % skip wristH link because of its fixed connection and negligible mass
    if i < 5
         limbero.links(i).m = limbero_urdf.Bodies{1,i+1}.Mass;
         p_cm = T_urdf2dh(:,:,i)*[limbero_urdf.Bodies{1,i+1}.CenterOfMass, 1]';
         limbero.links(i).r = p_cm(1:3);
         I = zeros(3,3);
         I_vector = limbero_urdf.Bodies{1,i+1}.Inertia;
         I(1,1) = I_vector(1); I(2,2) = I_vector(2); I(3,3) = I_vector(3); % xx, yy, zz
         I(1,2) = I_vector(4); I(2,1) = I(1,2); % xy, yx
         I(1,3) = I_vector(6); I(3,1) = I(1,3); % xz, zx
         I(2,3) = I_vector(5); I(3,2) = I(2,3); % yz, zy
         limbero.links(i).I = T_urdf2dh(1:3,1:3,i)*I*(T_urdf2dh(1:3,1:3,i))';
         limbero.links(i).G = n(i);
         limbero.links(i).Jm = Jm(i);
         limbero.links(i).qlim = limbero_urdf.Bodies{1,i+1}.Joint.PositionLimits;
    else
         if i == 5
            limbero.links(i).m = limbero_urdf.Bodies{1,i+1}.Mass + limbero_urdf.Bodies{1,i+2}.Mass;
         else
            limbero.links(i).m = limbero_urdf.Bodies{1,i+2}.Mass;
         end
         p_cm = T_urdf2dh(:,:,i)*[limbero_urdf.Bodies{1,i+2}.CenterOfMass, 1]';
         limbero.links(i).r = p_cm(1:3);
         I = zeros(3,3);
         I_vector = limbero_urdf.Bodies{1,i+2}.Inertia;
         I(1,1) = I_vector(1); I(2,2) = I_vector(2); I(3,3) = I_vector(3); % xx, yy, zz
         I(1,2) = I_vector(4); I(2,1) = I(1,2); % xy, yx
         I(1,3) = I_vector(6); I(3,1) = I(1,3); % xz, zx
         I(2,3) = I_vector(5); I(3,2) = I(2,3); % yz, zy
         limbero.links(i).I = T_urdf2dh(1:3,1:3,i)*I*(T_urdf2dh(1:3,1:3,i))';
         limbero.links(i).G = n(i);
         limbero.links(i).Jm = Jm(i);
         limbero.links(i).qlim = limbero_urdf.Bodies{1,i+2}.Joint.PositionLimits;
    end

    % FRICTION, for now just random numbers
    % including friction in the model,Inverse and Direct dynamic resolution
    % with rne become much more complex and slow..
%     limbero.links(i).B = 0.1;
%     limbero.links(i).Tc = [0.4, -0.4];
end

%limbero.tool = transl(0,0,0.1);
%limbero.payload(limbero_urdf.Bodies{1,limbero_urdf.NumBodies}.Mass);


% 3D model of the links, 
% need a fix of STL reference frame in SolidWorks
%limbero.model3d = 'LIMBERO';

%% PLOT SINGLE LIMB  
figure('Name', 'LIMBERO LF LEG, SWINGING DH (Coxa-Gripper');
%limbero.plot3d([0, pi/2, 0, 0, 0, 0, 0], 'workspace', [-1 1 -1 1 -1 1], 'view', [30 30], 'scale', 0.6);
limbero.plot(qz, 'workspace', [-1 1 -1 1 -1 1], 'view', [30 30], 'scale', 0.6,'jaxes', 'nobase', 'noshadow', 'notiles');
title('Limbero+Grieel LF limb DH, swinging (q = qz)');

%% ROBOT 
%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%% COMPOSE 4 LIMBS %%%%%

g = [0, 0, -9.81]; % gravity vector 
% base dimension
l = 0.35; 
w = 0.35; 
% homogeneus transformation for base position
T_LF = transl(-l/2, w/2, 0)*trotz((pi/2+pi/4)*180/pi);
T_LH =  transl(-l/2, -w/2, 0)*trotz((pi+pi/4)*180/pi);
T_RH =  transl(l/2, -w/2, 0)*trotz((-pi/4)*180/pi);
T_RF =  transl(l/2, w/2, 0)*trotz((pi/4)*180/pi);

LF_leg = SerialLink(limbero, 'name', 'LF', 'gravity', g, 'base', T_LF);
LH_leg = SerialLink(limbero, 'name', 'LH', 'gravity', g, 'base', T_LH);
RH_leg = SerialLink(limbero, 'name', 'RH', 'gravity', g, 'base', T_RH);
RF_leg = SerialLink(limbero, 'name', 'RF', 'gravity', g, 'base', T_RF);

LIMBERO = [LF_leg, LH_leg, RH_leg, RF_leg]; 
qc = [0, 0, pi/2, 0, 0, 0, 0];

figure('Name', 'LIMBERO+GRIEEL q=qz')
hold on
for i = 1:length(LIMBERO)
    LIMBERO(i).plot(qc,'workspace', [-0.5 0.5 -0.5 0.5 -0.5 0.5], 'nobase', 'noshadow', 'notiles', 'view', [30 30], 'scale', 0.6); 
end

% Set axis limits manually to ensure the entire robot is visible
xlim([-1 1]);  % Set x-axis limits
ylim([-1 1]);  % Set y-axis limits
zlim([-1 1]);  % Set z-axis limits

% Set equal aspect ratio to avoid distortion
axis equal; 

% Add fictitious base, for now fixed 
patch([-l/2,-l/2,l/2,l/2], [w/2 -w/2 -w/2 w/2], [0 0 0 0], 'FaceColor', 'b', 'FaceAlpha', 0.8);

hold off

title('LIMBERO configuration');



