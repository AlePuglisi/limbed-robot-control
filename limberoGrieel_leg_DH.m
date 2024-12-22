clear all
close all
clc

%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%% LIMBERO LEG %%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%

%% URDF LIMBERO LEG MODEL %%
% MODEL IMPORT AND INITIALIZATION

% limbero leg model
limbero_urdf = importrobot("urdf/LEG_LFGRIPPER.urdf", 'MeshPath', 'lbr_description/meshes');
limbero_urdf.Gravity = [0 0 -9.81]; % set up gravity 

limbero_urdf.DataFormat = 'row'; 

% initialize configuartion vector
config0 = homeConfiguration(limbero_urdf); 

% show model in new configuration
figure('Name', 'Limbero URDF')
show(limbero_urdf, config0); 
title('Limbero+Grieel LF leg URDF (RigidBodyTree) (q = qz)');

% initialize joint space velocity for dynamics 
% q_dot = zeros(1,limb_dof); 
% q_dot(F2T_index) = pi/10; 

% dynamic tensors computation in current configuration 
% M = massMatrix(limbero_urdf, config0);
% C = velocityProduct(limbero_urdf, config0, q_dot)';
g = gravityTorque(limbero_urdf, config0)';
tau = zeros(7,1);

%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% SWING PHASE, DH MODEL %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% KINEMATIC PARAMETERS
% DH parameters LIMBERO+GRIEEL

N_link = 7; 
% initial offset taken from URDF
% F2T_init_offset = 1.364075222;
% T2E_init_offset = 0.2067211047;

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

alpha = [-pi/2,0,0,-pi/2,pi/2, -pi/2, 0]; %[in rad]
a = [a1,a2,a3,0,a5,a6, 0]; %[in m]
d = [0,0,0,0,d5,0,d7]; %[in m]

% Generate the link object associated with the robot model (DH param); 
L1 = Link('d', d(1), 'a', a(1), 'alpha', alpha(1)); % coxa
L2 = Link('d', d(2), 'a', a(2), 'alpha', alpha(2)); % femur
L3 = Link('d', d(3), 'a', a(3), 'alpha', alpha(3)); %tibia
L4 = Link('d', d(4), 'a', a(4), 'alpha', alpha(4), 'offset', -pi/2); % foot_end
L5 = Link('d', d(5), 'a', a(5), 'alpha', alpha(5)); % wristH
L6 = Link('d', d(6), 'a', a(6), 'alpha', alpha(6)); % wrist V
L7 = Link('d', d(7), 'a', a(7), 'alpha', alpha(7)); % driving 

L1.name = 'Coxa'; 
l2.name = 'Femur'; 
L3.name = 'Tibia';
L4.name = 'Limb_end'; 
L5.name = 'wristH';
L6.name = 'wristV';
L7.name = 'driving';
L= [L1 L2 L3 L4 L5 L6 L7];


%% INITIALIZE the robot
limbero = SerialLink(L);
limbero.name = 'LIMBERO';
limbero.gravity = [0;0;9.81]; %gravity acceleration vector expressed in the base frame 


%% CONVERSION OF DYNAMIC PROPERTY FROM URDF TO DH 
qz = zeros(1,limbero.n);

% coxa
T_dh_coxa = limbero.A(1, qz);
T_urdf_coxa = limbero_urdf.getTransform(config0, 'LF_coxa');
T_dh2urdf_coxa = T_dh_coxa.T\T_urdf_coxa; % = (T_dh_coxa.T)^-1*T_urdf_coxa,same meaning 
% femur
T_dh_femur = limbero.A(1:2, qz);
T_urdf_femur = limbero_urdf.getTransform(config0, 'LF_femur');
T_dh2urdf_femur = T_dh_femur.T\T_urdf_femur;
% tibia
T_dh_tibia = limbero.A(1:3, qz);
T_urdf_tibia = limbero_urdf.getTransform(config0, 'LF_tibia');
T_dh2urdf_tibia = T_dh_tibia.T\T_urdf_tibia;
% limb_end
T_dh_limb_end = limbero.A(1:4, qz);
T_urdf_limb_end = limbero_urdf.getTransform(config0, 'LF_limb_end');
T_dh2urdf_limb_end = T_dh_limb_end.T\T_urdf_limb_end;
% wristH_Link
T_dh_wristH = limbero.A(1:5, qz);
T_urdf_wristH = limbero_urdf.getTransform(config0, 'LF_wristV_Link');
T_dh2urdf_wristH = T_dh_wristH.T\T_urdf_wristH;
% wristV_Link
T_dh_wristV = limbero.A(1:6, qz);
T_urdf_wristV = limbero_urdf.getTransform(config0, 'LF_driving_Link');
T_dh2urdf_wristV = T_dh_wristV.T\T_urdf_wristV;
% driving_Link
T_dh_driving = limbero.A(1:7, qz);
T_urdf_driving = limbero_urdf.getTransform(config0, 'LF_gripper_Link');
T_dh2urdf_driving = T_dh_driving.T\T_urdf_driving;

T_dh2urdf = zeros(4, 4, limbero.n);
T_dh2urdf(:,:,1) = T_dh2urdf_coxa; 
T_dh2urdf(:,:,2) = T_dh2urdf_femur; 
T_dh2urdf(:,:,3) = T_dh2urdf_tibia; 
T_dh2urdf(:,:,4) = T_dh2urdf_limb_end; 
T_dh2urdf(:,:,5) = T_dh2urdf_wristH; 
T_dh2urdf(:,:,6) = T_dh2urdf_wristV; 
T_dh2urdf(:,:,7) = T_dh2urdf_driving; 

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


for i=1:limbero.n
    % skip wristH link because of its fixed connection and negligible mass
    if i < 5
         limbero.links(i).m = limbero_urdf.Bodies{1,i+1}.Mass;
         p_cm = T_dh2urdf(:,:,i)*[limbero_urdf.Bodies{1,i+1}.CenterOfMass, 1]';
         limbero.links(i).r = p_cm(1:3);
         I = zeros(3,3);
         I_vector = limbero_urdf.Bodies{1,i+1}.Inertia;
         I(1,1) = I_vector(1); I(2,2) = I_vector(2); I(3,3) = I_vector(3); % xx, yy, zz
         I(1,2) = I_vector(4); I(2,1) = I(1,2); % xy, yx
         I(1,3) = I_vector(6); I(3,1) = I(1,3); % xz, zx
         I(2,3) = I_vector(5); I(3,2) = I(2,3); % yz, zy
         limbero.links(i).I = T_dh2urdf(1:3,1:3,i)*I*(T_dh2urdf(1:3,1:3,i))';
         limbero.links(i).G = n(i);
         limbero.links(i).Jm = Jm(i);
         limbero.links(i).qlim = limbero_urdf.Bodies{1,i+1}.Joint.PositionLimits;
    else
         if i == 5
            limbero.links(i).m = limbero_urdf.Bodies{1,i+1}.Mass + limbero_urdf.Bodies{1,i+2}.Mass;
         else
            limbero.links(i).m = limbero_urdf.Bodies{1,i+2}.Mass;
         end
         p_cm = T_dh2urdf(:,:,i)*[limbero_urdf.Bodies{1,i+2}.CenterOfMass, 1]';
         limbero.links(i).r = p_cm(1:3);
         I = zeros(3,3);
         I_vector = limbero_urdf.Bodies{1,i+2}.Inertia;
         I(1,1) = I_vector(1); I(2,2) = I_vector(2); I(3,3) = I_vector(3); % xx, yy, zz
         I(1,2) = I_vector(4); I(2,1) = I(1,2); % xy, yx
         I(1,3) = I_vector(6); I(3,1) = I(1,3); % xz, zx
         I(2,3) = I_vector(5); I(3,2) = I(2,3); % yz, zy
         limbero.links(i).I = T_dh2urdf(1:3,1:3,i)*I*(T_dh2urdf(1:3,1:3,i))';
         limbero.links(i).G = n(i);
         limbero.links(i).Jm = Jm(i);
         limbero.links(i).qlim = limbero_urdf.Bodies{1,i+2}.Joint.PositionLimits;
    end
    % FRICTION, for now just random numbers!!
%     limbero.links(i).B = 0.1;
%     limbero.links(i).Tc = [0.4, -0.4];
end

%limbero.tool = transl(0,0,0.1);
%limbero.payload(limbero_urdf.Bodies{1,limbero_urdf.NumBodies}.Mass);


% 3D model need a fix..
%limbero.model3d = 'LIMBERO';

%% PLOT ROBOT 
figure('Name', 'LIMBERO LF LEG, SWINGING DH (Coxa-Gripper');
%limbero.plot3d([0, pi/2, 0, 0, 0, 0, 0], 'workspace', [-1 1 -1 1 -1 1], 'view', [30 30], 'scale', 0.6);
limbero.plot(qz, 'workspace', [-1 1 -1 1 -1 1], 'view', [30 30], 'scale', 0.6,'jaxes', 'nobase', 'noshadow', 'notiles');
title('Limbero+Grieel LF limb DH, swinging (q = qz)');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% CONTACT  PHASE, DH MODEL %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% KINEMATIC PARAMETERS
% DH parameters LIMBERO+GRIEEL
a1_contact = a6; 
a2_contact = a5; 
d3_contact = -d5; 
a4_contact = a3; 
a5_contact = a2; 
a6_contact = a1; 
d7_contact = -0.02; % half coxa heigth

a_contact = [a1_contact, a2_contact, 0, a4_contact, a5_contact, a6_contact, 0];
d_contact = [0, 0, d3_contact, 0, 0, 0, d7_contact];
alpha_contact = [-pi/2, pi/2, -pi/2, 0, 0, -pi/2, 0];

L1_contact = Link('d', d_contact(1), 'a', a_contact(1), 'alpha', alpha_contact(1)); % driving
L2_contact = Link('d', d_contact(2), 'a', a_contact(2), 'alpha', alpha_contact(2)); % wirstV
L3_contact = Link('d', d_contact(3), 'a', a_contact(3), 'alpha', alpha_contact(3)); % wristH
L4_contact = Link('d', d_contact(4), 'a', a_contact(4), 'alpha', alpha_contact(4),'offset', pi/2); % foot_end
L5_contact = Link('d', d_contact(5), 'a', a_contact(5), 'alpha', alpha_contact(5), 'offset', -pi/2); % tibia
L6_contact = Link('d', d_contact(6), 'a', a_contact(6), 'alpha', alpha_contact(6)); % femur
L7_contact = Link('d', d_contact(7), 'a', a_contact(7), 'alpha', alpha_contact(7)); % coxa 

L7_contact.name = 'Coxa'; 
l6_contact.name = 'Femur'; 
L5_contact.name = 'Tibia';
L4_contact.name = 'Limb_end'; 
L3_contact.name = 'wristH';
L2_contact.name = 'wristV';
L1_contact.name = 'driving';

L_contact = [L1_contact, L2_contact, L3_contact, L4_contact, L5_contact, L6_contact, L7_contact];

%% ROBOT INITIALIZATION
limbero_contact = SerialLink(L_contact);
limbero_contact.name = 'LIMBERO_{contact}';
limbero_contact.gravity = [0;0;9.81]; % gravity acceleration vector expressed in the base frame 

%% DYNAMIC IDENTIFICATION 
% conversion from URDF to DH

config_contact = [0, 0, pi/2, 0, 0, 0, 0];

% To make the transformation from URDF to DH, an additional intermediate
% transformation is needed, because of the different reference frame. 
% T_0_dh2urdf = (limbero_urdf.getTransform(config_contact, 'LF_gripper_Link')*transl(0,0,-d7))\limbero_urdf.getTransform(config_contact, 'LF_coxa');
T_0dh_coxaurdf_contact = limbero_contact.A(1:6,qz).T*trotz(pi*180/pi);

% coxa
T_dh_coxa_contact = T_0dh_coxaurdf_contact\limbero_contact.A(1:7, qz).T;
T_urdf_coxa_contact = limbero_urdf.getTransform(config_contact, 'LF_coxa');
T_dh2urdf_coxa_contact = T_dh_coxa_contact\T_urdf_coxa_contact;
% femur
T_dh_femur_contact = T_0dh_coxaurdf_contact\limbero_contact.A(1:6, qz).T;
T_urdf_femur_contact = limbero_urdf.getTransform(config_contact, 'LF_femur');
T_dh2urdf_femur_contact = T_dh_femur_contact\T_urdf_femur_contact;
% tibia
T_dh_tibia_contact = T_0dh_coxaurdf_contact\limbero_contact.A(1:5, qz).T;
T_urdf_tibia_contact = limbero_urdf.getTransform(config_contact, 'LF_tibia');
T_dh2urdf_tibia_contact = T_dh_tibia_contact\T_urdf_tibia_contact;
% limb_end
T_dh_limb_end_contact = T_0dh_coxaurdf_contact\limbero_contact.A(1:4, qz).T;
T_urdf_limb_end_contact = limbero_urdf.getTransform(config_contact, 'LF_limb_end');
T_dh2urdf_limb_end_contact = T_dh_limb_end_contact\T_urdf_limb_end_contact;
% wristH_Link
T_dh_wristH_contact = T_0dh_coxaurdf_contact\limbero_contact.A(1:3, qz).T;
T_urdf_wristH_contact = limbero_urdf.getTransform(config_contact, 'LF_wristV_Link');
T_dh2urdf_wristH_contact = T_dh_wristH_contact\T_urdf_wristH_contact;
% wristV_Link
T_dh_wristV_contact = T_0dh_coxaurdf_contact\limbero_contact.A(1:2, qz).T;
T_urdf_wristV_contact = limbero_urdf.getTransform(config_contact, 'LF_driving_Link');
T_dh2urdf_wristV_contact = T_dh_wristV_contact\T_urdf_wristV_contact;
% driving_Link
T_dh_driving_contact = T_0dh_coxaurdf_contact\limbero_contact.A(1, qz).T;
T_urdf_driving_contact = limbero_urdf.getTransform(config_contact, 'LF_gripper_Link');
T_dh2urdf_driving_contact = T_dh_driving_contact\T_urdf_driving_contact;

T_dh2urdf_contact = zeros(4, 4, limbero_contact.n);
T_dh2urdf_contact(:,:,7) = T_dh2urdf_coxa_contact; 
T_dh2urdf_contact(:,:,6) = T_dh2urdf_femur_contact; 
T_dh2urdf_contact(:,:,5) = T_dh2urdf_tibia_contact; 
T_dh2urdf_contact(:,:,4) = T_dh2urdf_limb_end_contact; 
T_dh2urdf_contact(:,:,3) = T_dh2urdf_wristH_contact; 
T_dh2urdf_contact(:,:,2) = T_dh2urdf_wristV_contact; 
T_dh2urdf_contact(:,:,1) = T_dh2urdf_driving_contact; 

% remember to flip motor joint property because of joint order in the contact
% model
n_contact = flip(n);
Jm_contact = flip(Jm);

p_cm_tilde = zeros(N_link,4);
p_cm = zeros(N_link, 3);

for i=1:limbero_contact.n
    % skip wristH link because of its fixed connection and negligible mass
    if i < 4
         limbero_contact.links(i).m = limbero_urdf.Bodies{1,10-i}.Mass;
         p_cm_tilde(i,:) = T_dh2urdf_contact(:,:,i)*[limbero_urdf.Bodies{1,10-i}.CenterOfMass, 1]';
         p_cm(i,:) = p_cm_tilde(i,1:3);
         limbero_contact.links(i).r = p_cm(i,:);
         I = zeros(3,3);
         I_vector = limbero_urdf.Bodies{1,10-i}.Inertia;
         I(1,1) = I_vector(1); I(2,2) = I_vector(2); I(3,3) = I_vector(3); % xx, yy, zz
         I(1,2) = I_vector(4); I(2,1) = I(1,2); % xy, yx
         I(1,3) = I_vector(6); I(3,1) = I(1,3); % xz, zx
         I(2,3) = I_vector(5); I(3,2) = I(2,3); % yz, zy
         limbero_contact.links(i).I = T_dh2urdf_contact(1:3,1:3,i)*I*(T_dh2urdf_contact(1:3,1:3,i))';
         limbero_contact.links(i).G = n(i);
         limbero_contact.links(i).Jm = Jm(i);
         limbero_contact.links(i).qlim = limbero_urdf.Bodies{1,10-i}.Joint.PositionLimits;
    else
         if i == 4
            limbero_contact.links(i).m = limbero_urdf.Bodies{1,10-i}.Mass + limbero_urdf.Bodies{1,10-i-1}.Mass;
         else
            limbero_contact.links(i).m = limbero_urdf.Bodies{1,9-i}.Mass;
         end
         p_cm_tilde(i,:) = T_dh2urdf_contact(:,:,i)*[limbero_urdf.Bodies{1,9-i}.CenterOfMass, 1]';
         p_cm(i,:) = p_cm_tilde(i,1:3);
         limbero_contact.links(i).r = p_cm(i,:);
         I = zeros(3,3);
         I_vector = limbero_urdf.Bodies{1,9-i}.Inertia;
         I(1,1) = I_vector(1); I(2,2) = I_vector(2); I(3,3) = I_vector(3); % xx, yy, zz
         I(1,2) = I_vector(4); I(2,1) = I(1,2); % xy, yx
         I(1,3) = I_vector(6); I(3,1) = I(1,3); % xz, zx
         I(2,3) = I_vector(5); I(3,2) = I(2,3); % yz, zy
         limbero_contact.links(i).I = T_dh2urdf_contact(1:3,1:3,i)*I*(T_dh2urdf_contact(1:3,1:3,i))';
         limbero_contact.links(i).G = n(i);
         limbero_contact.links(i).Jm = Jm(i);
         limbero_contact.links(i).qlim = limbero_urdf.Bodies{1,9-i}.Joint.PositionLimits;
    end
    % FRICTION, for now just random numbers,slow down a lot rne dynamics
    %  limbero.links(i).B = 0.1;
    %  limbero.links(i).Tc = [0.4, -0.4];
end

% limbero_contact.tool = transl(0.18, -0.18, 0);
% limbero_contact.payload(3.1152/4 + limbero_contact.links(7).m, [0.18, -0.18, 0]);
%limbero_contact.links(N_link).m =  limbero_contact.links(N_link).m + 3.1152/4;

%%  PLOT LIMBERO+GRIEEL contact model 
figure('Name', 'LIMBERO LF LEG, CONTACT DH (Gripper-coxa)')
limbero_contact.plot([0, 0, 0, 0, 0, 0, pi/4], 'workspace', [-1 1 -1 1 -1 1], 'view', [30 30], 'scale', 0.6,'jaxes', 'nobase', 'noshadow', 'notiles');
title('Limbero+Grieel LF limb DH, contact (q = qz)')
% revert z and y axis for a proper visualization
set(gca, 'Zdir', 'reverse');
set(gca, 'Ydir', 'reverse');

%% CHECK for consistency between URDF and DH models %%

FK_urdf = limbero_urdf.getTransform([pi/4 0 0 0 0 0 0], 'LF_gripper_Link');
FK_dh = limbero.fkine([pi/4 0 0 0 0 0 0]);

%% EXPERIMENT on custom defined dynamic computation functions
r_cm = zeros(N_link,3);
masses = zeros(N_link,1);
offset = zeros(N_link,1);

r_cm_contact = zeros(N_link,3);
masses_contact = zeros(N_link,1);
offset_contact = zeros(N_link,1);

for i=1:N_link
    r_cm(i,:) = limbero.links(i).r;
    masses(i) = limbero.links(i).m;
    offset(i) = limbero.links(i).offset;

    r_cm_contact(i,:) = limbero_contact.links(i).r;
    masses_contact(i) = limbero_contact.links(i).m;
    offset_contact(i) = limbero_contact.links(i).offset;
end

g0 = Gravity_compensation(a,d,alpha, offset, r_cm, masses, qz, 0);
g0_contact = Gravity_compensation(a_contact,d_contact,alpha_contact, offset_contact, r_cm_contact, masses_contact, qz, 1);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% INDIPENDENT JOINT CONTROL TUNING %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% To tune the controller:

% 1) first an estimation of inertia seen from each joint has to be done: 
% this can be done using an average inertia when changing one joint,
% maintaining the other fixed: 

% initialize vector of joint inertia around rotation axis
Jl = zeros(1,7); 
Jl_contact = zeros(1,7);
Jl_avg = zeros(1,7);

% generate the vector of all  meaningfull combinations 
% meaningful configurations joint limit 
q_lim = [-pi, -pi/6, -3/4*pi, -pi/2, -pi, -pi/4, 0;
          pi,  pi/2,    pi/2,  pi/2,  pi,  pi/2, 0];
q_lim_contact = [ 0, -pi/6, -pi, -pi/4, -pi/6, -pi/2, -pi; 
                  0,  pi/6,  pi,  pi/4, pi*3/2, pi/6,  pi];

delta_q = pi/6; % configuration space (C-space) discretization 
q0 = {};
q0_contact = {};

% generate cell of array of joint configuartion in the discretized C-space 
for i=1:N_link
    q0(i) = {q_lim(1,i):delta_q:q_lim(2,i)};
    q0_contact(i) = {q_lim_contact(1,i):delta_q:q_lim_contact(2,i)}; 
end

% select random index  
n_test = 50;
for i=1:N_link
    k(i,:) = randi(length(q0{1,i}), 1, n_test);
    k_contact(i,:) = randi(length(q0_contact{1,i}), 1, n_test);
end

% create a 1x7 vector collecting the average Jli(i=1,..,7), both for
% limb in contact and in swing model

for i=1:n_test
    q = [q0{1,1}(k(1,i)),q0{1,2}(k(2,i)),q0{1,3}(k(3,i)),q0{1,4}(k(4,i)),q0{1,5}(k(5,i)),q0{1,6}(k(6,i)),q0{1,7}(k(7,i))];
    q_contact = [q0_contact{1,1}(k_contact(1,i)),q0_contact{1,2}(k_contact(2,i)),q0_contact{1,3}(k_contact(3,i)),q0_contact{1,4}(k_contact(4,i)),q0_contact{1,5}(k_contact(5,i)),q0_contact{1,6}(k_contact(6,i)),q0_contact{1,7}(k_contact(7,i))];
    M(:,:,i) = limbero.inertia(q);
    M_contact(:,:,i) = limbero_contact.inertia(q_contact);
    
%     figure('Name', 'LIMBERO LF LEG, SWING DH (coxa-Gripper)')
%     limbero.plot(q, 'workspace', [-1 1 -1 1 -1 1], 'view', [30 30], 'scale', 0.6,'jaxes', 'nobase', 'noshadow', 'notiles');
%     title('Limbero+Grieel LF limb DH, swinging (q_test)');
%     figure('Name', 'LIMBERO LF LEG, CONTACT DH (Gripper-coxa)')
%     limbero_contact.plot(q_contact, 'workspace', [-1 1 -1 1 -1 1], 'view', [30 30], 'scale', 0.6,'jaxes', 'nobase', 'noshadow', 'notiles');
%     title('Limbero+Grieel LF limb DH, contact (q_test)')
%     set(gca, 'Zdir', 'reverse');
%     set(gca, 'Ydir', 'reverse');
%    
%     pause()
%     close all

    for j=1:size(M,1)
         Jl(i,j) = M(j,j,i);
         Jl_contact(i,j) = M_contact(8-j, 8-j,i);
         Jl_avg(i,j) = (Jl_contact(i,j) + Jl(i,j))/2; 
    end
end

for i=1:N_link
    Jl_avg_joints(i) = sum(Jl_avg(:,i))/size(Jl_avg,1);
end

% 2) use identifyed inertia to tune indipendent joint PPI controller 
%% CONTROLLER TUNING 
w_cp = 10;      % position loop cut-off frequency [rad/s] ~2 Hz
w_cv = 10*w_cp; % velocity loop cut-off frequency [rad/s] ~20 Hz

% SPEED CONTROL 
Tiv = 1/(0.15*w_cv); % at least one decade after wcv
Tiv = Tiv*10;        % velocity loop Integral time  

mu = zeros(1,7);
kpv = zeros(1,7); 

for i=1:limbero.n
    mu(i) = 1/(Jl_avg_joints(i)); % robot joint model TF gain 
    kpv(i) = w_cv/mu(i);          % velocity loop proportional gain 
end

% POSITION CONTROL 
kpp = w_cp; % position controller proportional gain 

% additional filtering
w_lpf = 15;              % joint reference Low pass pre-filter cut-off [rad/s]
T_lpf = 1/(w_lpf*2*pi);  % joint reference Low pass pre-filter time constant 

w_vel = 50;              % derivative realizability pole frequency (for speed estimation) 
T_vel = 1/(w_vel*2*pi);  % derivative realizability pole time contant (for speed estimation) 

Tc = Tiv*0.1;            % Back-calculation anti wind-up time constant
L = 30;                  % Desaturation anti-wind up parameter 

% Conversion of PPI position-velocity cascade control as PID position control 
KP = zeros(1,7);
TD = zeros(1,7);
TI = zeros(1,7);
KD = zeros(1,7);
KI = zeros(1,7);

for i=1:limbero.n
    KP(i) = kpv(i)*(kpp+1/Tiv);         % position proportional gain 
    TD(i) = kpv(i)/KP(i);               % position derivative time constant 
    TI(i) = KP(i)*Tiv/(kpp*kpv(i));     % position integral time constant 

    KD(i) = KP(i)*TD(i);                % position derivative gain
    KI(i) = KP(i)/TI(i);                % position integral gain
end

%% Save usefull data in a mat file: 
save('limbero_data', 'limbero', 'limbero_contact', 'limbero_urdf');






