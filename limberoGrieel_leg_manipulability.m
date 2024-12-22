clear 
close all 
clc 

% Load limbero model (DH, DH contact, URDF) in the workspace 
load('limbero_data');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% ELLIPSOID MANIPULABILITY %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% LIMBERO SWING 

% Define useful varaibles: 
N_link = limbero.n;
q0 = zeros(1, N_link);
q1 = [0 -pi*2/3 pi*2/3 -pi/4 0 0 0];

q = q0;

% Compute jacobian matrix 
J = limbero.jacob0(q);

% Core of velocity manipulability ellipsoid 
E = J*J'; 

Et = E(1:3, 1:3);
Er = E(4:6, 4:6);

% check if decoupling of rotation/translation can be numerically meaningful   
coupling_ratio = norm(E(1:3, 4:6))/(norm(Et)+ norm(Er));

% plot robot and ellipsoid to check if meaningful
figure('Name', 'LIMBERO LF LEG, SWINGING DH (Coxa-Gripper');
limbero.plot(q, 'workspace', [-1 1 -1 1 -1 1], 'view', [30 30], 'scale', 0.6,'jaxes', 'nobase', 'noshadow', 'notiles');
title('Limbero+Grieel LF limb DH, swinging (q = qz)');

% rototransalte to tool frame 
T0 = limbero.fkine(q);
Tool_translation = T0.t; 
Tool_rotation = T0.R; 
% rescale for visualization purpose 
%new_Et = Tool_rotation*Et*Tool_rotation'*0.005^2; 
%new_Et = Et*0.005^2;

% plot ellipsoid 
hold on 
plot_ellipse(Et,Tool_translation, 'edgecolor', 'r');

%% Alternative decomposition in translation/rotation
% jacobian decomposition
Jt = J(1:3,:);
Jr = J(4:6,:);
% ellipsoid decomposition 
Et = Jt*Jt';
Er = Jr*Jr'; 

% plot robot 
figure('Name', 'LIMBERO LF LEG, SWINGING DH (Coxa-Gripper');
limbero.plot(q, 'workspace', [-1 1 -1 1 -1 1], 'view', [30 30], 'scale', 0.6,'jaxes', 'nobase', 'noshadow', 'notiles');
title('Limbero+Grieel LF limb DH, swinging (q = qz)');
% plot ellipsoid 
new_Et = Tool_rotation*Et*Tool_rotation'; 
hold on 
plot_ellipse(Et,Tool_translation, 'edgecolor', 'r');

%% LIMBERO CONTACT 
