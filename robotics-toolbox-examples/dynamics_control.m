clc 
close all 
clear 

%% ROBOTIXTOOLBOX by Peter Cork, Chapter 9: 
%   DYNAMICS AND CONTROL 

%% EQUATIONS OF MOTION 
% Q = M(q)qdd + C(q,qd)qd + F(qd) + G(q) + J(q)'*g
% Q := vector of generalized actuator forces
% M := joint-space inertia matrix
% C := Coriolis and Centripertal coupling matrix
% F := friction force 
% G := gravity load 
% g := wrench at the end-effector 
mdl_puma560; 

% DYNAMIC when NOT MOVING, qd=0, qdd=0
% inverse dynamic solved using recursive newton-euler method 
Q = p560.rne(qn, qz, qz); % joint space (pose, velocity, acceleration) 
% with 0 velocity and acceleration, this correspond just to grevity effect
% if in fact we force gravity vector to 0, the actuator force is 0: 
Q_z = p560.rne(qz, qz, qz, [0 0 0]'); 

% rne method can be used also for a trajectory 
q = jtraj(qz, qr, 10); %from qz to qr in 10 steps
Q_jtraj = p560.rne(q, 0*q, 0*q); 
% each row represent the generalized forces at joint space corresponding to
% the joint trajectory point 

% DYNAMIC when MOVING qd!=0
% with unitary speed on joint 1
Q_qd1 = p560.rne(qn, [1 0 0 0 0 0], qz, [0 0 0]'); 
% a force is exerted on all joints due to friction and couplings

% Dynamics description of links: 
p560.links(1).dyn;

%% ANALYSIS OF EACH DYNAMICAL TERM: 
% Further description of the dynamic model expression..

%% GRAVITY TERM (gravity tensor)
% in order to extract the G(q) term from the equation of motion
% 1) imposing zero speed and acceleration at configurtaion of
% interset 
gravload0 = p560.rne(qn, qz, qz); 
% 2) with a direct method 
gravload_earth = p560.gravload(qn); 

% gravitational computation is based on the gravity property of the robot
g = p560.gravity; 
% So, we can set the desired value of gravity, for example to set lunar
p560.gravity = p560.gravity/6; % 1/6 of earth gravity 
gravload_lunar = p560.gravload(qn); 

disp(['earth gravload : ', num2str(gravload_earth), newline, 'lunar gravload: ', num2str(gravload_lunar)]);

% we can also test the change in the gravload when we set the robot
% upside-down
p560.base = trotx(pi*180/pi); 
p560.plot(qn, 'notiles', 'perspective', 'noshadow')
gravload_lunar_down = p560.gravload(qn);
disp(['lunar gravload upside-down: ', num2str(gravload_lunar_down)]);

% reset gravity and base pose: 
mdl_puma560; 
% as can be seen, the gravutational load change with the pose
% and the effect on each joint is intutive
gravload_qs = p560.gravload(qs); 
gravload_qr = p560.gravload(qr);

% Analyzing in detail the variation of gravitational torque in q2,q3, we
% can plot it in 3d. This is foundamental to understand the design of the
% robot motor and the torque requirements
[Q2, Q3] = meshgrid(-pi:0.1:pi, -pi:0.1:pi); 
for i=1:numcols(Q2)
    for j=1:numcols(Q3)
        g = p560.gravload([0 Q2(i,j) Q3(i,j) 0 0 0]); 
        g2(i,j) = g(2); 
        g3(i,j) = g(3); 
    end
end
figure('Name', 'Gravitational Load on joint q2,q3')
subplot(1,2,1)
surfl(Q2, Q3, g2); 
xlabel('q2')
ylabel('q3')
zlabel('g2')
title('g2(q2,q3)')
subplot(1,2,2)
surfl(Q2, Q3, g3);
xlabel('q2')
ylabel('q3')
zlabel('g3')
title('g3(q2,qlinks_foot_up3)')

% this give us a good insight on the gravitational load requirements on
% each joint, expecially joint2 (shoulder) has a wider range in Nm than
% joint3, meaning that shoulder joint need a more powerfull motor

%% INERTIA MATRIX (inertia tensor) 
% representing the inertia effect, with diagonal element Mii as inertia
% seen by joint i, while Mij the effect on joint i torque of acceleration
% on joint j (M is symmetric)
M = p560.inertia(qn);
% in nominal configuration, inertia M11 and M22 are higher, waist and
% shoulder has to move much heavier links 

% investigating by a 3d plot as done before: 
[Q2, Q3] = meshgrid(-pi:0.1:pi, -pi:0.1:pi); 
for i=1:numcols(Q2)
    for j=1:numcols(Q3)
        M = p560.inertia([0 Q2(i,j) Q3(i,j) 0 0 0]); 
        M11(i,j) = M(1,1); 
        M12(i,j) = M(1,2);
        M22(i,j) = M(2,2); 
    end
end

figure('Name', 'Inertia Matrix when joint 2,3 moves')
subplot(1,3,1)
surfl(Q2,Q3,M11); 
xlabel('q2')
ylabel('q3')
zlabel('M_{11}')
subplot(1,3,2)
surfl(Q2,Q3,M12); 
xlabel('q2')
ylabel('q3')
zlabel('M_{12} = M_{21}')
subplot(1,3,3)
surfl(Q2,Q3,M22); 
xlabel('q2')
ylabel('q3')
zlabel('M_{22}')

% obviously M22 depends on q3 but not on q2. This is the inertia seen at
% joint2..

%% CORIOLIS MATRIX 
% Cii is the effect related to centripetal torque proportional to qdi^2,
% while Cij represent the proportional terms of coriolis torques for
% qid*qjd. 
qd = 0.5*[1 1 1 1 1 1]; % 0.5 rad/s in each joint 
C = p560.coriolis(qn,qd); % in nominal configuration qn 

% coriolis and centripetal joint torques: 
tau_c = C*qd'; 

%% PAYLOAD EFFECT 
% when the manipulator has to move a payload with its joints, the dynamic
% performances decrease, a robotic arm in fact has a rated maximum payload,
% which depend a lot on joint motor limits
mdl_puma560;
gravload = p560.gravload(qn);
M = p560.inertia(qn); % ienrtia matrix without load 

% first set a payload of 2.5 Kg, shifted by 0.1m on the z axis
p560.payload(2.5, [0,0,0.1]); 

% then, compute the new inertia matrix 
M_loaded = p560.inertia(qn); 
% compared to the old inertia matrix without load: 
M_ratio = M_loaded ./ M; 

% also gravload is affected: 
gravload_ratio = p560.gravload(qn) ./ gravload;

%% FORCE ON THE BASE 
% when robot moves, wrench is applyed on its base, we can compute the
% required wrench to keep it fixed. 
% This term is computed by the rne method directly
mdl_puma560;
[Q,g] = p560.rne(qn,qz,qz);
% as can be seen, the vector g in the z axis represent the force to
% counteract weight 
robot_mass = sum([p560.links.m]);

%% DYNAMIC MANIPULABILITY MEASURE 
% extension of the manipulability as performance measure, we can reffer to
% the performance in terms of: 
% ability to accelerate in different cartesian directions
J = p560.jacob0(qn); 
M = p560.inertia(qn); 
Mx = (J * inv(M) * inv(M)' * J'); 

% considering the translational component of acceleration performance 
Mx = Mx(1:3, 1:3);
figure('Name', 'Dynamic Manipulability ellipsoid')
plot_ellipse(Mx, 'edgecolor', 'b');
xlabel('x')
ylabel('y')
zlabel('z')
grid on 
v = [-5 -2 5];
[caz,cel] = view(v);

% with radii 
radii = sqrt(eig(Mx)); 
% and condition number 
cond_qn = min(radii)/max(radii); 

% the dynamic measure is used by the asada option of the manipulability
% meausure computation 
m_dyn_qn = p560.maniplty(qn, 'asada'); 

%% FORWARD DYNAMIC
close all 
clc
clear 

mdl_puma560; 
% compuation of joint acceleration given joint toqrues, by inverting the
% dynamic equation of motion
% method 1: 
q = qn; 
qd = qz; 
Q = qz; 
qdd = p560.accel(q,qd,Q); 
% then by double integration you can obtain the evolution of q(t)

% similarly, this can be dine with the method fdyn
% t_max = 1;
% [t, joint_traj, joint_vel] = p560.fdyn(t_max,@my_torque_function, qn,qz);
% 
% function tau = my_torque_function(robot, t, q, qd)
%     tau = zeros(1,robot.n); 
% end

% another way is to rely on simulink scheme: 
r = sim('sl_ztorque'); % run simulation 
t = r.find('tout'); % save time
q = r.find('yout'); % save joint trajectory 

% plot the trajectory 
figure('Name', 'Forward Dynamic')
subplot(1,2,1)
p560.plot(q, 'perspective', 'noshadow', 'notiles'); 
subplot(1,2,2)
plot(t, q(:,1:3)); 
grid on
legend('q1', 'q2', 'q3')
xlabel('t')
ylabel('q_{i}(t)')




