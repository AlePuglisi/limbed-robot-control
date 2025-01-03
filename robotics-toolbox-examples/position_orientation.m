clc
clear 
close all 

%% ROBOTIXTOOLBOX by Peter Cork, Chapter 2: 
%   Representing Position and Orientation 

%% POSE in 2D
t1 = [1 2]; % Translation vector 
R1 = rotz(30);
R1 = R1(1:2, 1:2); % Rotation matrix as 30deg rotation 

T1 = SE2(R1,t1); % Homogeneus transformation matrix 

figure('Name','Homogeneus Transformation in 2D')
trplot2(T1, 'frame', '1', 'color', 'b') % specify T1 with 'frame' label as 1 and 'color' as blue 'b'
grid on 
hold on 

% define a second homogeneus transfromation
T2 = SE2(eye(2), [2 1]);


trplot2(T2, 'frame', '2', 'color', 'r')

% compose the two transformations
T3 = T1*T2; %first apply T1 and then T2

trplot2(T3, 'frame', '3', 'color', 'g')

% Non commutativeness of homogeneus transformation concatenation 
T4 = T2*T1; %first apply T2 and then T1


trplot2(T4, 'frame', '4', 'color', 'c')

% plot a point in the world frame 
P = [3; 2]; 
plot_point(P, '*', 'color', 'b');
% express P on frame {1}
P1 = T1.inv * P; % as euclidean point 2x1

% HERE IS WRONG.. something different in the TOOLBOX.
% P1_a = h2e(T1.inv * P); % as euclidean point 2x1, using h2e(homogeneus to euclidean) 
% P1_b = homtrans(T1.inv, P); % directly as homogeneus transform 
% % express P on frame {2}
% P2 = homtrans(inv(T2).tform, P);

axis([0 5 0 5]);

%% POSE in 3D

%% ROTATION as ORTHONORMAL ROTATION MATRIX SO(3) 
R = rotx(90); % 90 deg around x         axis

figure('Name','Rotation in 3D with R')
trplot(R, 'perspective') % use perspectve to view the 3d plot in perspective 
% tranimate(R) % to animate the rotation 

% non commutative rotation in 3D proven:
R2 = rotx(90)*roty(90); % first 90 around x then 90 around y
R3 = roty(90)*rotx(90); % first 90 around y then 90 around x

trplot(R2, 'perspective', 'frame', '1', 'color', 'r')
%tranimate(R2, 'color', 'r') % to visualize rotation in real time 
hold on 
trplot(R3, 'perspective', 'frame', '2', 'color', 'b')
%tranimate(R3, 'color', 'b') % to visualize rotation in real time 

%% THREE ANGLE REPRESENTATION 

% ZYZ Euler (phi, theta, psi)
% figure('Name','Rotation in 3D with Euler/RPY')
R4 = rotz(0.1*180/pi)*roty(0.2*180/pi)*rotz(0.3*180/pi); %explicitly computed
R4 = eul2r(0.1*180/pi, 0.2*180/pi, 0.3*180/pi); % concisely 

% inverse problem,from R to euler ZYZ can be done easily 
gamma = tr2eul(R4); 

% when y axis rotation theta is negative, anyway the inverse tr2eul return
% a theta positive and a different set of ZYZ angles. Not unique
% correspondance between R and ZYZ angles. 
% ALSO, theta=0 is a singularity. 
R = eul2r(0.1*180/pi, 0 , 0.3*180/pi); 
gamma2 = tr2eul(R); % only psi!=0, singularity. 

% RPY angles convention: 
R5 = rpy2r(0.1*180/pi, 0.2*180/pi, 0.3*180/pi, 'zyx'); % RPY angles to rotation matrix 
gamma3 = tr2rpy(R5); %inverse problem 

%% TWO VECTOR REPRESENTATION 

% Represent orientation using the EndEffector approach and normal vector,
% which completely specify the Rotation m,atrix
a = [1 0 0]';
o = [0 1 0]'; 
R = oa2r(o,a);

%% ROTATION ABOUT ARBITRARY ANGLE, AXIS-ANGLE REPRESENTATION 

R = rpy2r(0.1*180/pi, 0.2*180/pi, 0.3*180/pi, 'zyx');
[theta, v] = tr2angvec(R);
% the axis-angle representation consists of chosing one axis around which
% rotate the frame of the defined angle. The axis correspeond to the
% eigenvector of the rotation matrix with unitary eigenvalue.
% for the opposite operation: 
R6 = angvec2r(pi/2, [1 1 1]);
% visual representation
% figure('Name','Axis-Angle Representation')
% plot3([0 1],[0 1],[0 1]);
% hold on
% grid on 
% axis([-2 2 -2 2 -2 2]);
% tranimate(R6);

%% UNIT QUATERNION 

q = UnitQuaternion( rpy2tr(pi/2*180/pi, pi/2*180/pi, 0, 'zyx'));
quaternion_norm = q.norm; % which is 1 by definition
R = q.R; % convert quaternion into Rotation Matrix 
figure('Name', 'Unit Quaternion Rperesentation')
q.plot()

% several operations with quaternions has been overwritten..

%% ROTATION + TRANSLATION: 
% 2 possible main representation: 
% 1) transaltion vector + unit quaternion 
% 2) by Homogeneus transformation matrix T, as Rotation matrix and
% translation vector: 
 T = transl(1,0,0) * trotx(pi/2*180/pi) * transl(0,1,0);

 figure('Name', 'Rotation + Translation')
 trplot(T, 'perspective')

 % extract R and t: 
 R = t2r(T);
 t = transl(T);


