clc
clear 
close all

%% ROBOTIXTOOLBOX by Peter Cork, Chapter 7: 
%   ROBOT ARM KINEMATICS, PART 1 (7.1-7.4)

%% DH convention 
% to specify the Robot kinematic structure as a chain of links
% interconnected by joints: 

L = Link([0, 0.1, 0.2, pi/2, 0]); %theta="0"(placeholder), d, a, alpha, R type (1 for P) 

% to access the homogeneus transformation matrix
A_01 = L.A(0.5); % homogeneus transformation for theta=0.5 rad

% also, its kinematic DH parameters can be easily accessed: 
L_type = L.type; 
L_a = L.a; 
L_offset = L.offset; % offset added to the joint variable before computing link transformation
% if we define an offset and then recompute A, we get different values
% (for example, with 0.5 offset, we obtain A(0) same as previous A(0.5)
L.offset = 0.5; 
A_01_offsetted = L.A(0); 

%% SerialLink and FORWARD KINEMATIC (FK)
% Find end-effector pose given joint configuration 
% EX1: 2 LINK RR Robot
% define robot links
L1(1) = Link([0 0 1 0]); 
L1(2) = Link([0 0 1 0]); 
% create robot structure as series of links 
two_link = SerialLink(L1, 'name', 'two link'); 

% some properties of SerialLink object allow to investigate the structure: 
n_joints = two_link.n; 
links = two_link.links; 
two_link_clone = SerialLink(two_link, 'name', 'clone'); 

% solving FK for a given configuration of (q1,q2,..)
T_0 = two_link.fkine([0 0]); % solve fkine for q1=0, q2=0, return the homogeneus transformation matrix
% as can be seen R_0 = I (same orientation as base frame) 
% t_0 = [2 0 0]' because of each link length of 1

T_1 = two_link.fkine([pi/4 -pi/4]); 

% joint cohordinate are treated as row vectors by convention

% VISUALIZATION: we can easily visualize the robot structure 
figure('Name', '2 link RR with q1=0, q2=0')
two_link.plot([0 0], 'jaxes', 'notiles'); % in (0,0) config
axis([-2 3 -2 3 -2 3])

figure('Name', '2 link RR with q1=pi/4, q2=-pi/4')
two_link_clone.plot([pi/4 -pi/4], 'jaxes', 'notiles'); 
axis([-2 3 -2 3 -2 3])

% EX2: 6-axis Robot RRRRRR (6R)
mdl_puma560 % function script which create a p560 SerialLink object as robot

% also it initialize some standard configuration variables 
% qz := all 'zero' angle config
% qr := 'ready' configuration, vertical and straight
% qs := 'stretch' with straight and horizontal arm 
% qn := 'nominal' in dexterous working pose 
% VIUSALIZE 
figure('Name', 'p560 configuration')
q = qn; 
if q == qz
p560.plot(qz, 'jaxes', 'notiles')
title('zero config'); 
end
if q == qr
p560.plot(qr, 'jaxes', 'notiles')
title('ready pose config'); 
end
if q == qs
p560.plot(qs, 'jaxes', 'notiles')
title('stretch pose config'); 
end
if q == qn
p560.plot(qn, 'jaxes', 'notiles')
title('nominal pose config'); 
end

% we can solve FK also here as before
Tz_p560 = p560.fkine(qz); 

% CHANGE TOOL 
% notice that frame N is on the wrist axis intersection point.
% we can define a further transforma
% tion to reach tool tip, using tool
% attribute of the SerialLink object, which is initialized as eye(4)
p560.tool = transl(0, 0, 0.2); 
% once redefined the tool tip as Tool Center Point, fkine has a new
% solution
Tz_p560_newtool = p560.fkine(qz); 

% CHANGE BASE
% also the base frame 0 is considered in the intersection between shoulder
% and waist, considering the 30inch tall pedestal (30*0.0254 m)
p560.base = transl(0, 0, 30*0.0254); 
Tz_p560_newtoolandbase = p560.fkine(qz); %consider now correct base and tool 

% we can even change the base like hanging from the ceiling
% do this by rotate x axis by 180 deg and translate bu 3m
% p560.base = transl(0, 0, 3)*trotx(pi*180/pi);
% Tz_p560_ceiling = p560.fkine(qz);

% figure('Name', 'p560 ceiling')
% p560.plot(qz, 'jaxes', 'notiles')

% fkine support also joint angle as time series or trajectories 
% representing each timestep by a different row 
% [q(t0); 
% q(t1); ....; 
% q(tn)] 
% than, if we apply fkine(q) it returns a 3D matrix of dimension 4x4xN
% later on this will be further explained.. 

%% INVERSE KINEMATICS (IK)

%% CLOSED FORM SOLUTION
% Find joint configuration given desired end-effector pose 
T = p560.fkine(qn); % FK for nominal configuration 

% to solve puma560 IK, we can use the ikine6s method, defined for 6axis
% robt with spherical wrist (if not so, problem occurs). And the resolution
% is based on closed form solution
qn_IK = p560.ikine6s(T); 

figure('Name', 'IK Solutions for Puma560')
p560.plot(qn_IK, 'jaxes', 'notiles' ); 
Tn_IK = p560.fkine(qn_IK); 

% even if qn_IK!=qn, they have the same T, this is because the same pose
% can be obtained from different configurations, we have multiple IK
% solutions.. in these case one is right-handed, the other left-handed
% we can force the type of solution by: 
qn_IK_ru = p560.ikine6s(T,'ru'); % 'ru':= Right-handed, elbow Up
% now this is the same as qn

% obviously, some poses are not feasible because 
% 1) not reachable
%q_out = p560.ikine6s(transl(3,0,0)); % WARNING
% 2) meet manipulator singularity, for Puma560 this occurs when q5=0, in
% this case q4,q6 are alligned and with ikine we can identify only q4+q6,
% but the individual value is arbitrary 
q_sing = [0 pi/4 pi 0.1 0 0.2];
q_sing_IK = p560.ikine6s(p560.fkine(q_sing), 'ru'); 
% they are different, but q4+q6 is the same

%% NUMERICAL SOLUTION 
% using the method ikine(T,q_init) we can solve the inverse kinematic for
% any robot manipulator, even when close to singularities, obviously slower
% than ikine6s because not optimized. And it doesn't give you the freedom
% of chosing the type of ik solution like elbow/wrist..

T_num = p560.fkine(qn); 
qn_IKnum = p560.ikine(T); 
% again qn_IKnum != qn, but the same T
T_numIK = p560.fkine(qn_IKnum); 
% We can specify the initial configuration from which solve the IK, in
% order to make a consistent solution, useful for consistent trajectory
% generation 
qn_IKnumq0 = p560.ikine(T, 'q0', [0 pi/4 -pi/2 0 0 0]); %in this way, elbow-up configuration is reached 
figure('Name', 'IK Numerical Solutions for Puma560')
p560.plot(qn_IKnumq0, 'jaxes', 'notiles' ); 

%% UNDER-ACTUATED MANIPULATOR 
% using the two link manipulator: 
mdl_twolink;
% desired EE pose 
T = transl(0.4, 0.5, 0.6); 
twolink.base = eye(4); 
% this robot has a task space subspace of R^2, we don't have any ftask reedom on
% orientation and z position, then we need somehow to specify to IK solver
% that only x,y position has to be considered when solving the tx,ty,tz,
% rx,ry,rz pose request given by T. This is done using a boolean mask 
q_xy = twolink.ikine(T, 'q0',[0 0],'mask', [1 1 0 0 0 0]); 
T_IK = twolink.fkine(q_xy); 

figure('Name', 'Under-Actuated IK')
twolink.plot(q_xy,'view','top', 'notiles')

%% REDUNDANT MANIPULATOR 
% defining a puma560 over a mobile platform on xy, we obtain a
% configuration space higher than the task space. 
mdl_puma560; 
L1 = Link([0 0 0 -pi/2 1]); 
L1.qlim = [0; 2]; 
L2 = Link([-pi/2 0 0 pi/2 1]); 
L2.qlim = [0; 2];
platform = SerialLink([L1, L2] , 'base', troty(-pi/2*180/pi), 'name', 'platform'); 


T_platform = platform.fkine([1,2]); 

% concatenate platform and puma: 
%p8 = platform*p560; % method 1
%p8 = SerialLink([platform, p560]); % method 2

% but we want to put the torso joint up at 30 inches on the base..
% set pedestal height for Puma560 on the moving base: 
% (we do this using d(2) because reset the base is not possible) 
p560.links(1).d = 30 * 0.0254; 
p8 = SerialLink([platform, p560]);

figure('Name', 'Zero Configuration')
p8.plot(zeros(1,8), 'notiles', 'perspective', 'jaxes'); 

T = transl(0.5, 1.0, 0.7) * troty(-pi/2*180/pi); % move tool to (0.5,1,0.7), with tool pointing downward in xz-plane
%q_IK = p8.ikine(T, 'q0', zeros(1,8));

% figure('Name', 'Puma560 on Mobile Base')
% p8.plot(q_IK, 'notiles', 'perspective'); 
% ERRORS IN THE RESOLUTION.. SOMETHING OCCURS

%% TRAJECTORIES
% generate a trajectory from point A to point B, this can be generated at
% joint space or cartesian space

%% JOINT_SPACE MOTION 
% considering end-effector moving between two poses
mdl_puma560; 
T1 = transl(0.4,0.2, 0) * trotx(pi*180/pi); 
T2 = transl(0.4, -0.2, 0) * trotx(pi/2*180/pi); 

% corresponding to the joint state
q1 = p560.ikine6s(T1);
q2 = p560.ikine6s(T2); 

% time vector from 0 to 2 with 0.05 sec step 
t = [0:0.05:2]'; 

% one method to define trajectory in joint space: 
q = mtraj(@tpoly, q1, q2, t); % or @lspb for trapezioidal 
% q = jtraj(q1,q2,t) % is even more efficent way to do the same
% [q, qd, qdd] = jtrat(q1, q2, t) % to get both position, velocity and
% acceleration 

% more concise way: apply it directly to the robot
q = p560.jtraj(T1, T2, t); % this doesn't even need the usage of IK 
% PLOT p560 ANIMATION: 
figure('Name', 'p560 Joint Trajectory Animation'); 
p560.plot(q, 'perspective', 'notiles'); 

% also plot other characteristic of the generated joint space trajectory
figure('Name', 'Joint Space Trajectory')
subplot(2,2,1)
qplot(t, q); %qplot allow to plot all the angles in time
subplot(2,2,2)
T = p560.fkine(q); % obtain a 4x4xlenght(t) matrix (1x41 SE3) 
p = transl(T); % extract translation x,y,z
plot(t, p); 
legend('x', 'y', 'z'); 
xlabel('Time step')
ylabel('position')
grid on 

% plotting motion in xy-plane
subplot(2,2,3)
plot(p(:,1), p(:,2)); 
xlabel('x')
ylabel('y')
grid on 
axis([0 0.7 -0.2 0.2])
P1 = [0.4; 0.2]; 
P2 = [0.4; -0.2];

plot_point(P1, 'o', 'color', 'b', 'label', 'P1');
plot_point(P2, '*', 'color', 'r', 'label', 'P2');

% rpy angles during trajectory 
subplot(2,2,4)
plot(t, tr2rpy(T)); 
legend('roll', 'pitch', 'yaw')
xlabel('Time step')
ylabel('RPY angle')
grid on 

% it is clear that here we don't have any control on the end-effector pose
% during trajectory, if we need to avoid obstacles, a cartesian space
% trajectory has to be used. 

% similar functionality can be realized by using >> sl_jspace (in Simulink)

%% CARTESIAN MOTION 
% similarly as befpre, it is solved using function ctraj directly on
% SerialLink object, taking as input the poses T1,T2 and the length of the
% time vector
Ts = ctraj(T1, T2, length(t)); 

figure('Name', 'p560 Cartesian Trajectory Animation'); 
p560.plot(p560.ikine6s(Ts), 'perspective', 'notiles'); 
% as desired, now the trajectory is linear in the cartesian space 

figure('Name', 'Cartesian Space Trajectory')

subplot(2,2,1)
qplot(t, p560.ikine6s(Ts)); %qplot allow to plot all the angles in time

subplot(2,2,2)
p = transl(Ts); % extract translation x,y,z
plot(t, p); 
legend('x', 'y', 'z'); 
xlabel('Time step')
ylabel('position')
grid on 

% plotting motion in xy-plane
subplot(2,2,3)
plot(p(:,1), p(:,2)); 
xlabel('x')
ylabel('y')
grid on 
axis([0 0.7 -0.2 0.2])
P1 = [0.4; 0.2]; 
P2 = [0.4; -0.2];

plot_point(P1, 'o', 'color', 'b', 'label', 'P1');
plot_point(P2, '*', 'color', 'r', 'label', 'P2');

% rpy angles during trajectory 
subplot(2,2,4)
plot(t, tr2rpy(Ts)); 
legend('roll', 'pitch', 'yaw')
xlabel('Time step')
ylabel('RPY angle')
grid on 

%% MOTION THROUGH A SINGULARITY 
% handle trajectory passingsubplot(2,2,1)close to singularity 
T1 = transl(0.5, 0.3, 0.44) * troty(pi/2*180/pi); 
T2 = transl(0.5, -0.3, 0.44) * troty(pi/2*180/pi); 

% cartesian trajectory 
Ts = ctraj(T1, T2, length(t));
qc_ik6s = p560.ikine6s(Ts); % using specific ikine6s formulation  
qc_ik = p560.ikine(Ts); % using generalized numerical IK solution 
% joint trajectory 
q = p560.jtraj(T1,T2,t); 

figure('Name', 'Trajectory close to Singularity')
subplot(2,2,1)
qplot(t,qc_ik6s)
title('cspace ikine6s IK solution')

subplot(2,2,2)
qplot(t, qc_ik)
title('cspace ikine IK solution')

subplot(2,2,3)
qplot(t,q)
title('joint-space motion')

subplot(2,2,4)
m_ik6s = p560.maniplty(qc_ik6s); 
m_ik = p560.maniplty(q); 
plot(t,[m_ik6s,m_ik]); 
title('Manipulability measure')
legend('Cartesian path', 'Joint space path')
grid on

%% CONFIGURATION CHANGE 
% when we cant to change the configuartion without moving the pose of the
% end effector, we cannot do this by planning trajectory in cartesian
% space. Instead we plan in joint space solving the IK imposing different
% configurations 

% end-effector pose 
T = transl(0.4, 0.2, 0)*trotx(pi*180/pi); 

% right-handed config
qr = p560.ikine6s(T, 'ru'); 
% left-handed config
ql = p560.ikine6s(T, 'lu'); 

% same pose but different configuration specifyed
q = jtraj(qr, ql, t); % solving trajectory in joint space directly 

figure('Name', 'IK for COnfiguration Change')
subplot(1,2,1)
qplot(t, q); 
title('Joint space motion')
subplot(1,2,2)
p560.plot(q, 'perspective', 'notiles')
title('Puma560 Animation')

%% KINEMATICS CONTINUE IN THE SCRIPT 
% /robot_arm_kin_advanced.m



