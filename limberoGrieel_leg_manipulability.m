clear 
close all 
clc 

% Load limbero model (DH, DH contact, URDF) in the workspace 
load('limbero_data');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% ELLIPSOID MANIPULABILITY %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% LIMBERO SWING %%

% Define useful varaibles: 
N_link = limbero.n;
q0 = zeros(1, N_link);
q1 = [0 -pi*2/3 pi*2/3 -pi/4 0 0 0];

q = q0;

% Compute jacobian matrix 
J = limbero.jacob0(q);

% Core of velocity manipulability ellipsoid 
E = J*J'; 

Et1 = E(1:3, 1:3);
Er1 = E(4:6, 4:6);

% check if decoupling of rotation/translation can be numerically meaningful   
coupling_ratio = norm(E(1:3, 4:6))/(norm(Et1)+ norm(Er1));

% plot robot and ellipsoid to check if meaningful
figure('Name', 'LIMBERO LF LEG,SWINGING(Coxa-Gripper), MANIPULABILITY');
limbero.plot(q, 'workspace', [-1 1 -1 1 -1 1], 'view', [30 30], 'scale', 0.6,'jaxes', 'nobase', 'noshadow', 'notiles');
title('Limbero+Grieel LF limb DH, swinging (q = qz), Manipulability');

% rototransalte to tool frame 
T0 = limbero.fkine(q);
Tool_translation = T0.t; 
Tool_rotation = T0.R; 
% rescale for visualization purpose 
%new_Et = Tool_rotation*Et*Tool_rotation'*0.005^2; 
%new_Et = Et*0.005^2;

% plot ellipsoid 
hold on 
plot_ellipse(Et1,Tool_translation, 'edgecolor', 'r');

%% Direct way through roboticstoolbox: 
figure('Name', 'LIMBERO LF LEG,SWINGING(Coxa-Gripper), MANIPULABILITY');
limbero.plot(q, 'workspace', [-1 1 -1 1 -1 1], 'view', [30 30], 'scale', 0.6,'jaxes', 'nobase', 'noshadow', 'notiles');
title('Limbero+Grieel LF limb DH, swinging (q = qz), Manipulability');
% plot ellipsoid 
limbero.vellipse(q, 'fillcolor', 'b', 'edgecolor', 'w', 'alpha', 0.5);

%-----------------------------%%
%%    LIMBERO CONTACT         %%

