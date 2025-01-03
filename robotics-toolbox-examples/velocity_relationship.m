clc
clear  
close all

%% ROBOTIXTOOLBOX by Peter Cork, Chapter 8: 
%   VELOCITY RELATIONSHIP 

%% MANIPULATOR JACOBIAN / GEOMETRICAL JACOBIAN 
% it relate the vector of joint velocirt with the spatial velocity of the
% manipulator end effectorm, by a matrix relation 

mdl_puma560; 

T0 = p560.fkine(qn); 
dq = 1e-6; % small perturbation of joint1
Tp = p560.fkine(qn + [dq 0 0 0 0 0]); % new pose after perturbation

dTdq1 = (Tp-T0) / dq; % derivative of transformation
% as can be seen, it is not a proper homogeneus transformation matrix
% the last column in the translation part define the relation between
% changes in q1 and changes on end effector pose, which correspond to
% row 1-3 of column 1 of J ( translation velocity relation)

Tp = p560.fkine(qn + [0 dq 0 0 0 0]); % perturbation of joint2
dTdq2 = (Tp-T0) / dq; 

dRq1 = dTdq1(1:3,1:3); % extract DR part 
R = T0.R; 
S = dRq1*R'; 
% then we can find the end effector amgular velocity wx,wy,wz inverting as
wdq1 = vex(S);% this correspond to row 4-6 of column 1 of J

% doing the same for q2
dRq2 = dTdq2(1:3,1:3); % extract DR part 
S = dRq2*R'; 
% then we can find the end effector amgular velocity wx,wy,wz inverting as
wdq2 = vex(S); % this correspond to row 4-6 of column 2 of J (rotation velocity relation)
% this represent relationship between joint velocity and angular velocity
% of end effector

% those elements computed will correspond to columns of the jacobian
% matrix, that can be computed given the robot configuration q: 
J0_geom = p560.jacob0(qn); % 'jacob0' is the jacobian in the world coordinate frame


%% TRANSFORM VELOCITY BETWEEN COORDINATE FRAMES 
% it is possible to relate the velocity in two coordinate frames with a
% jacobian matrix: 
T = transl(1,0,0)*troty(pi/2*180/pi); % homogeneus transformation between two frames 
J_velAB = tr2jac(T);  % jacobian related to the frames relation 

vA = [1 0 0 0 0 0]'; % in frame A, it is a unitary velocity on xa direction 
vB = J_velAB*vA; % in frame B, after the transformation, it correspond to unitary velocity in zb
% simple transforation between frames!

%% JACOBIAN IN THE END-EFFECTOR COORDINATE FRAME 
% jacob0 relate joint velocity to end-egffector velocity in the world
% frame, if we are interested in the relation expressing velocoty in the
% end-effecor frame instead, we use jacobe
Jn_geom = p560.jacobe(qn);

%% ANALYTICAL JACOBIAN 
% in terms of angular velocity, instead of relating wx,wy,wz of the end
% effector frame, it is possible to consider the time derivative of the
% minimal representation: 
% this is done using the jacobian between representation: 
B = rpy2jac(0.1, 0.2, 0.3);
% and computing the overall analytical jacobian 
J0_an = p560.jacob0(qn,'eul');

%% JACOBIAN CONDITION AND MANIPULABILITY 
% the inverse jacobian can be used to map the desired end-effector motion
% (velocity) to joint velocity. qdot=J^-1*v
% when det(J) = 0, inversion is not possible, the robot is said to be in a
% kinematic singularity!
J0_qr = p560.jacob0(qr);
% as can be seen, it is rank deficient: 
rJ_qr = rank(J0_qr); % 5 < 6

% we can check the singularity analysis
jsingu(J0_qr) % q6 depends on q4
figure('Name', 'Puma560 Singularity Analysis')
p560.plot(qr, 'notiles', 'noshadow', 'perspective', 'jaxes');
title('Puma560 in qr');
pause

% close to singular configuration, we observe big requested velocity
q = qr; 
q(5) = 5*pi/180; % 5 deg angle of q5, a bit far from singularity
p560.plot(q, 'notiles', 'noshadow', 'perspective', 'jaxes');
title('Puma560 close to qr')

J0_q = p560.jacob0(q); 
% inverting differential kinematic: 
qd = inv(J0_q)*[0 0 0.1 0 0 0]'; % achieve 0.1 m/s in z direction 
% very big requested speed of joints.. problematic!
d_q = det(J0_q); % the determinant in fact is small 

% this bad condiditioning correspond to a big condition number 
c_q = cond(J0_q); 

% not all motions are bad conditioned, a rotation around y axis is feasible
% without critical joint velocity 
qd_w = inv(J0_q)*[0 0 0 0 0.2 0]'; % 0.2 rotation around y axis 

% we can graphically visualize this condition by the manipulator ellipsoid 
Jp_q = J0_q(1:3,:); 
Jo_q = J0_q(4:6, :); 

figure('Name', 'Manipulability Ellipsoid on q close to qr')
subplot(1, 3, 1)
p560.plot(q, 'notiles', 'noshadow', 'perspective', 'jaxes');
title('Puma560 close to qr')

subplot(1,3,2)
title('Translational')
plot_ellipse(Jp_q*Jp_q', 'edgecolor', 'b')
xlabel('x')
ylabel('y')
zlabel('z')
grid on 
v = [-5 -2 5];
[caz,cel] = view(v);

subplot(1,3,3)
title('Rotational')
plot_ellipse(Jo_q*Jo_q', 'edgecolor', 'r')
xlabel('x')
ylabel('y')
zlabel('z')
grid on 
v = [-5 -2 5];
[caz,cel] = view(v);

%as global index of manipulability, we can rely on the manipulability index
m_qr = p560.maniplty(qr, 'yoshikawa'); 

% making the same analysis in a different configuratyion qn
p560_copy = SerialLink(p560, 'name', 'puma2');
J0_qn = p560_copy.jacob0(qn);
Jp_qn = J0_qn(1:3,:); 
Jo_qn = J0_qn(4:6, :); 

figure('Name', 'Manipulability Ellipsoid on qn')
subplot(1, 3, 1)
p560_copy.plot(qn, 'notiles', 'noshadow', 'perspective', 'jaxes');
title('Puma560 at qn')

subplot(1,3,2)
title('Translational')
plot_ellipse(Jp_qn*Jp_qn', 'edgecolor', 'b')
xlabel('x')
ylabel('y')
zlabel('z')
grid on 
v = [-5 -2 5];
[caz,cel] = view(v);

subplot(1,3,3)
title('Rotational')
plot_ellipse(Jo_qn*Jo_qn', 'edgecolor', 'r')
xlabel('x')
ylabel('y')
zlabel('z')
grid on 
v = [-5 -2 5];
[caz,cel] = view(v);

m_qn = p560.maniplty(qn, 'yoshikawa'); 
disp(['Manipulability index: \n qn: ',num2str(m_qn), '\qr: ', num2str(m_qr)]);

%% FORCE RELATIONSHIP 
%% TRANSFORMING FORCES BETWEEN FRAMES 
J_AB = tr2jac(transl(2, 0, 0)); % jacobian relating frames A to B (from B to A) 
F = J_AB'*[0 3 0 0 0 0]'; % force in frame A

%% STATICS
mdl_puma560; 

p560.plot(qn, 'perspective', 'noshadow', 'notiles'); 
% if a force of 20N along world y direction is applyed at the end effector,
% it correspond to a joint force of: 
tau_20y = p560.jacob0(qn)' * [0 20 0 0 0 0]'; 
% if instead it is applyed along x
tau_20x = p560.jacob0(qn)' * [20 0 0 0 0 0]'; 

