clc
clear  
close all 

%% ROBOTIXTOOLBOX by Peter Cork, Chapter 3: 
%   Time and Motion, TRajectory generation 

%% ONE-DIMENSIONAL TRAJECTORY
%% 5th ORDER TRAJECTORY

% One-dimensional trajectories: 
% often 5th order polynomials are used to define trajectories, ensuring
% continuity over velocity and acceleration, plus contrained boundary
% conditions. 

s = tpoly(0,1,50); %s0 = 0, sf = 1, points = 50 (Quintic polynomial)
figure('Name', 'Quintic Polynomial Position s(t)')
plot(s);
xlabel('Time')
ylabel('s(t)')
grid on 

% we can even store velocity and acceleration
% with 0 initial velocity and acceleration
[s1, s1d, s1dd] = tpoly(0,1,50);

% with 0.5 initial velocity and 0 final velocity, while 0 acceleration
[s2, s2d, s2dd] = tpoly(0,1,50,0.5,0); 

% PLOT trajectories: 
% s1(t) and its derivative
figure('Name', 'Quintic Polynomial Position,Velocity,Acceleration s(t), sd(t), sdd(t)')
subplot(3,2,1);
plot(s1);
title('0 initial velocity and acceleration')
grid on 
xlabel('Time')
ylabel('s1(t)')
subplot(3,2,3);
plot(s1d)
grid on 
xlabel('Time')
ylabel('s1d(t)')
subplot(3,2,5);
plot(s1dd)
grid on 
xlabel('Time')
ylabel('s1dd(t)')

% s2(t) and its derivative
subplot(3,2,2);
plot(s2)
title('non 0 initial velocity')
grid on 
xlabel('Time')
ylabel('s2(t)')
subplot(3,2,4);
plot(s2d)
grid on 
xlabel('Time')
ylabel('s2d(t)')
subplot(3,2,6);
plot(s2dd)
grid on 
xlabel('Time')
ylabel('s2dd(t)')

% average velocity is much less then the maxixmum one, mantained for few
% time instants..
avg_vel_percentage = mean(s1d)/max(s1d); % only 52% of the peak

% we desire to mantain max velocity for as much time as possible, to
% guarantee fast motion and exploit actuation speed. 

%% HYBRID TRAJECTORY, "almost" TRAPEZIOIDAL VELOCITY
% LINEAR SEGMENT with PARABOLIC BLEND

[s3, s3d, s3dd] = lspb(0, 1, 50); % trapezioidal, without further specifications
max_nominal = max(s3d); % by default, but we can ovveride the speed 
[s4, s4d, s4dd] = lspb(0, 1, 50, 0.025); % set max velocity as 0.025 
[s5, s5d, s5dd] = lspb(0, 1, 50, 0.035); % set max velocity as 0.035
% NOTICE that the lspb is a constrained trajectory, maximum velocity has to
% be chosen consistently

% PLOT trajectories: 
% s3(t) and its derivative
figure('Name', 'Trapezoidal Polynomial Position,Velocity,Acceleration s(t), sd(t), sdd(t)')
subplot(3,2,1);
plot(s3);
title('default maximum velocity')
grid on 
xlabel('Time')
ylabel('s3')
subplot(3,2,3);
plot(s3d)
grid on 
xlabel('Time')
ylabel('s3d(t)')
subplot(3,2,5);
plot(s3dd)
grid on 
xlabel('Time')
ylabel('s3dd(t)')

% s3,s4,s5(t) and its derivative
subplot(3,2,2);
plot(s3)
hold on 
plot(s4)
plot(s5)
hold off
legend('nominal', '0.025', '0.035')
title('different maximum velocity')
grid on 
xlabel('Time')
ylabel('s(t)')
subplot(3,2,4);
plot(s3d)
hold on 
plot(s4d)
plot(s5d)
hold off
grid on 
xlabel('Time')
ylabel('sd(t)')
subplot(3,2,6);
plot(s3dd)
hold on 
plot(s4dd)
plot(s5dd)
hold off
grid on 
xlabel('Time')
ylabel('sdd(t)')

%% MULTI DIMENSIONAL TRAJECTORY 
% in robotics, both in mobile robot and manipulator, you define the
% trajectory for each DOF as smooth trajectory, being it just position or
% the full pose. 
% we can achieve trajectory interpolation for more DOF as: 

% 5th order polynomial, from (0,2) to (1,-1) in 50 steps
[x_p, x_pd, x_pdd] = mtraj(@tpoly, [0 2], [1 -1], 50);
% linear segment parabolic blend, from (0,2) to (1,-1) in 50 steps
[x_t, x_td, x_tdd] = mtraj(@lspb, [0 2], [1 -1], 50);

% PLOT trajectories: 
% s1(t) and its derivative
figure('Name', 'Position,Velocity,Acceleration x(t), xd(t), xdd(t)')
subplot(3,2,1);
plot(x_p);
title('5th order polynomial')
legend('x1', 'x2')
grid on 
xlabel('Time')
ylabel('xp(t)')
subplot(3,2,3);
plot(x_pd)
grid on 
xlabel('Time')
ylabel('xpd(t)')
subplot(3,2,5);
plot(x_pdd)
grid on 
xlabel('Time')
ylabel('xpdd(t)')

% s2(t) and its derivative
subplot(3,2,2);
plot(x_t)
title('non 0 initial velocity')
grid on 
legend('x1', 'x2')
xlabel('Time')
ylabel('xt(t)')
subplot(3,2,4);
plot(x_td)
grid on 
xlabel('Time')
ylabel('xtd(t)')
subplot(3,2,6);
plot(x_tdd)
grid on 
xlabel('Time')
ylabel('xtdd(t)')

%% MULTI-SEGMENT INTERPOLATION
% whenever we want to specify the trajectory through "via points", so as a
% sequence of intermediary point for each dof trajevctory. this is an
% over-constrained problem in which only some parameters can be specifyed. 
via = [ 4, 1; 4, 4; 5, 2; 2, 5]; % intermediate point (4,1), (4,4), ..
% matrix of trajectoryes
q1 = mstraj(via, [2,1], [], [4, 1], 0.05, 0); 
% via points, max speed, segment duration (Only one of vel or time can be assigned freely), initial point, sample interval, acceleration time
q2 = mstraj(via, [2,1], [], [4, 1], 0.05, 1); % if specify t_acc, increasing it becomes smoother 

% plot
figure('Name', 'Multi-Segment Interpolation')
subplot(1,2,1)
title('t_{acc} not specifyed')
xlabel('Time')
ylabel('x1,x2')
plot(q1)
legend('x1', 'x2'); 
grid on

subplot(1,2,2)
title('t_{acc} = 1, smoother')
xlabel('Time')
ylabel('x1,x2')
plot(q2)
legend('x1', 'x2'); 
grid on 

%% INTERPOLATION OF ORIENTATION IN 3D

% In principple, mstraj could be used to interpolate rpy 3-angle
% representation, but it is not an ideal way to interpolate between two
% desired rotations, because it generates unexpected rotation moovement not
% predictable.. other ways to perform orientation trajectory interpolation
% exists: 

% using RPY angles interpolation
% we rely on linear interpolation between roll-pitch-yaw angle: 
R0 = rotx(-pi/2*180/pi) * roty(-pi/2*180/pi);
R1 = rotx(pi/2*180/pi) * roty(pi/2*180/pi);

% convert into RPY representation 
rpy0 = tr2rpy(R0, 'xyz');
rpy1 = tr2rpy(R1, 'xyz');

% interpolate as 5th order trajectory 
rpy = mtraj(@tpoly, rpy0, rpy1, 50); 

% plot animation 
% ERROR, THERE IS SOME PROBLEM IN THE CONVERSION RPY AND VICEVERSA...
figure('Name', 'rpy interpolation')
trplot(R0, 'perspective', 'color', 'b', 'frame', '0'); 
hold on
trplot(R1, 'perspective', 'color', 'g', 'frame', '1');
tranimate(rpy2r(rpy, 'xyz') , 'perspective', 'color', 'r'); 
% this has some problems close to singularity 


% we can do the same using QUATERNION
q0 = UnitQuaternion(R0); 
q1 = UnitQuaternion(R1); 

% smoother interpolation, done using spherical linear interpolation
q = interp(q0, q1, [0:49]' /49); 

figure('Name', 'Quaternion interpolation')
trplot(R0, 'perspective', 'color', 'b', 'frame', '0'); 
hold on
trplot(R1, 'perspective', 'color', 'g', 'frame', '1');
% tranimate(q, 'perspective', 'color', 'r'); 
q.animate('color', 'r')

%% CARTESIAN MOTION
% interpolate changes both in poition and orientation 

% 1) At first try, it is possible to interpolate between two homogeneus
% transformation matrix, with quaternion interpolation: 
T0 = transl(0.4, 0.2, 0) * trotx(pi*180/pi); 
T1 = transl(-0.4, -0.2, 0.3) * troty(pi/2*180/pi) * trotz(-pi/2*180/pi); 

Ts = trinterp(T0, T1, [0:49]/49); % generate 50 points between T0 and T1

figure('Name', 'Cartesian Motion')
trplot(T0, 'perspective', 'color', 'b', 'frame', '0'); 
hold on
trplot(T1, 'perspective', 'color', 'g', 'frame', '1');
tranimate(Ts, 'color', 'r'); 

% as we can see by plotting translation and rotation
t = transl(Ts); 
rpy = tr2rpy(Ts, 'xyz'); 

figure('Name', 'Cartesian Motion with Linear Interpolation')
subplot(1,2,1)
plot(t); 
title('translation')
xlabel('Time step')
ylabel('x,y,z')
legend('x', 'y', 'z')
grid on

subplot(1,2,2)
plot(rpy); 
title('rotation')
xlabel('Time step')
ylabel('r,p,y')
legend('roll', 'pitch', 'yaw')
grid on

% 2) for a better interpolation, in order to guarantee continuous velocity
% and acceleration interpolation, we can specify the vector of normalized
% distances as tpoly or lspb
Ts_smooth = trinterp(T0, T1, lspb(0,1,50)); 
% Ts_smooth = ctraj(T0,T1, 50) % works in the same way as shortcut
t_smooth = transl(Ts_smooth);
rpy_smooth = tr2rpy(Ts_smooth, 'xyz'); 

figure('Name', 'Cartesian Motion Smooth')
trplot(T0, 'perspective', 'color', 'b', 'frame', '0'); 
hold on
trplot(T1, 'perspective', 'color', 'g', 'frame', '1');
tranimate(Ts_smooth, 'color', 'r'); 

figure('Name', 'Cartesian Motion with Linear and Trapezioidal Interpolation')

subplot(2,2,1)
plot(t); 
title('standard')
xlabel('Time step')
ylabel('x,y,z')
legend('x', 'y', 'z')
grid on
subplot(2,2,3)
plot(rpy); 
xlabel('Time step')
ylabel('r,p,y')
legend('roll', 'pitch', 'yaw')
grid on

subplot(2,2,2)
plot(t_smooth); 
title('smooth')
xlabel('Time step')
ylabel('x,y,z')
legend('x', 'y', 'z')
grid on
subplot(2,2,4)
plot(rpy_smooth); 
xlabel('Time step')
ylabel('r,p,y')
legend('roll', 'pitch', 'yaw')
grid on

%% TIME VARYING COORDINATE FRAMES
% how to represent rotation of a coordinate frame in time? 
% we refer to the skew symmetric operation, through the angular rotation
% vector w, (wx, wy, wz) representative of body rotation in time

w = [1 2 3]; 
S = skew(w); %skew symmetric matrix of w=(wx,wy,wz)
w_inv = vex(S); 

% Notice that infinitesimal angular changes are commutative: 
Rdelta1 = rotx(0.001*180/pi) * roty(0.002*180/pi) * rotz(0.003*180/pi); 
Rdelta2 = roty(0.002*180/pi)* rotx(0.001*180/pi) * rotz(0.003*180/pi); 
% Rdelta1 = Rdelta2 as can be seen

rotDelta = vex(Rdelta1 - eye(3,3))'; 

% for incremental motion: 
T0 = transl(1,2,3)*trotx(1*180/pi)*troty(1*180/pi)*trotz(1*180/pi); 
T1 = T0*transl(0.01, 0.02, 0.03)* trotx(0.001*180/pi)*troty(0.002*180/pi)*trotz(0.003*180/pi); 

% incremental displacement: 
d = tr2delta(T0, T1); 

% final displacement, closer to T1 except for small error, as DT*T0
Tf = delta2tr(d)*T0; 
