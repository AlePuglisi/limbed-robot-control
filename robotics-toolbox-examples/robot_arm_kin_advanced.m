clc
clear
close all

%% ROBOTIXTOOLBOX by Peter Cork, Chapter 7: 
%   ROBOT ARM KINEMATICS, PART 2 (7.5-7.7)

%% JOINT VARIABLE OFFSET

% the zero configuration imposed by the DH convention is not always the
% best one, in-fact for example for the pum560 arm robot the qz zero config
% is not so intuitive. We can decide to set a q0 offeset on the joint
% variable, which will be used during IK and FK
mdl_puma560; 
figure('Name', 'Puma560 Zero Configuration')
p560.plot(qz, 'perspective', 'notiles', 'jaxes'); 
title('No offset on q2')

p560.links(2).offset = pi/2; % set theta02=90 offset 

pause

p560.plot(qz, 'perspective', 'notiles', 'jaxes');
title('pi/2 offset on q2')
% as can be seen, it is different from the previous qz without offset

%% DETERMINE DH PARAMETER INTUITIVELY

% It is possible to define the robotic structure in a more intutive way, as
% sequence of rotation and translationby a chosen axis. This is specifyed
% by a string and then factorized by the toolbox it allow to create a
% SerialLink object in the workspace: 
L = [1, 0.4, 0.7, 0.1, 1.2, 0.3, 0.3]; % link lenght 
L1 = L(1); L2 = L(2); L3=L(3); L4=L(4); L5=L(5); L6=L(6); L7=L(7);
s = 'Tz(L1) Rz(q1) Ry(q2) Ty(L2) Tz(L3) Ry(q3) Tx(L6) Ty(L4) Tz(L5) Rz(q4) Ry(q5) Rz(q6)'; 
dh = DHFactor(s); 

cmd = dh.command('puma'); 
robot = eval(cmd); 
pause
figure('Name', 'Alternative Robot definition (no DH)')
robot.plot(qz, 'perspective', 'notiles', 'jaxes'); 
title('Determine DH param')

%% APPLICATION EXAMPLE 1: 
% DRAWING LETTER ON A PLANE 
% first define the letter as a sequence of via-points 
clear 
clc

path = [1 0 1; 1 0 0; 0 0 0; 0 2 0; 1 2 0; 1 2 1; 0 1 1; 0 1 0; 1 1 0; 1 1 1];
pause
close all
figure('Name', 'Draw Letter')

plot3(path(:,1), path(:,2), path(:,3), 'color', 'k', 'LineWidth',2)
grid on
axis([-3 3 -3 3 -0.5 2])
% Letter E path
p = mstraj(path, [0.5 0.5 0.3], [], [2 2 2], 0.02, 0.2); 
Tp = transl(0.1*p); %scale up the letter dimension, from 1mx2m to 20cmx19xm
Tp = homtrans(transl(0.4,0,0), Tp); % move the origin of the letter
pause

p_new = zeros(size(Tp,3), 3);
for i=1:size(Tp,3)
p_new(i,1:3) = Tp(1:3,4,i)';
end
plot3(p_new(:,1), p_new(:,2), p_new(:,3), 'color', 'k', 'LineWidth',0.05)
hold on 
title('rescaled path')
axis([-0.5 0.9 -0.4 0.5 -0.2 0.8])
grid on 

% load the robot model
mdl_puma560;
p560.tool = trotx(pi*180/pi); %make the tool reference frame pointing z-axis down
% compute IK for the desired path: 
q = p560.ikine6s(Tp); 
p_tool = zeros(size(q,1), 3);
first_time = 1; 
for i=1:size(q,1)
    p560.plot(q(i,:),'perspective', 'notiles', 'nojoints')
    T = p560.fkine(q(i,:));
    p_tool(i,1:3) = T.t';
    if p_tool(i,3) == 0
        if first_time == 1
            p_tool_plane(1,1:3) = p_tool(i,1:3);
            first_time = 0;
        else
            p_tool_plane(end+1,1:3) = p_tool(i,1:3);
        end
        plot3(p_tool_plane(:,1), p_tool_plane(:,2), p_tool_plane(:,3), 'color', 'r', 'LineWidth',0.05)
    end
end

figure('Name', 'Drawing Analysis')
subplot(1,2,1)
t = [0.02:0.02:numrows(p)*0.02]';
qplot(t,q)

subplot(1,2,2)
plot(p_tool(:,1), p_tool(:,2)); 
xlabel('x')
ylabel('y')

%% APPLICATION EXAMPLE 2: 
% SIMPLE WALKING ROBOT WITH 4 LEGS 
clc
clear 

% KINEMATICS
% to avoid the definition of DH paramters, we can use the techique
% explained before to directly derive the DH frames from an intuitive
% description as sequence of rotation and translation: 
% s = 'Rz(q1) Rx(q2) Ty(L1) Rx(q3) Tz(L2)'; 
% dh = DHFactor(s);
% 
L1 = 0.1; L2 = 0.1; 
% leg = eval(dh.command('leg')); 
% qz = zeros(1,length(leg.links));
% 
% figure('Name', 'Single Leg')
% leg.plot(qz, 'nobase', 'noshadow', 'notiles'); 
% set(gca, 'Zdir', 'reverse') %set z axis downward

% by DH defintion 
close all 
a = [0 L1 -L2]; 
alpha = [1.571 0 0];
d = [0 0 0]; 

l1 = Link('d', d(1), 'a', a(1), 'alpha', alpha(1), 'offset', pi/2);
l2 = Link('d', d(2), 'a', a(2), 'alpha', alpha(2));
l3 = Link('d', d(3), 'a', a(3), 'alpha', alpha(3), 'offset', -pi/2);

leg = SerialLink([l1, l2, l3], 'name', 'leg'); 
leg.tool = trotz(pi/2*180/pi)*trotx(-pi/2*180/pi)*trotz(-pi/2*180/pi);
qz = zeros(1,length(leg.links));

figure('Name', 'Single Leg')
leg.plot(qz, 'nobase', 'noshadow', 'notiles', 'jaxes');
title('Zero Configuration')
set(gca, 'Zdir', 'reverse') %set z axis downward

% check proper joint actuation, moving each joint 
pause
q1 = [0.2, 0, 0];
t1 = transl(leg.fkine(q1)); 
leg.plot(q1, 'nobase', 'noshadow', 'notiles', 'jaxes');
title('q1 motion (expected motion in xy plane)')
set(gca, 'Zdir', 'reverse') %set z axis downward
pause
q2 = [0, 0.2, 0]; 
t2 = transl(leg.fkine(q2)); 
leg.plot(q2, 'nobase', 'noshadow', 'notiles', 'jaxes');
title('q2 motion (expected motion in yz plane)')
set(gca, 'Zdir', 'reverse') %set z axis downward
pause
q3 = [0, 0, 0.2];
t3 = transl(leg.fkine(q3)); 
leg.plot(q3, 'nobase', 'noshadow', 'notiles', 'jaxes');
title('q3 motion (expected motion in yz plane)')
set(gca, 'Zdir', 'reverse') %set z axis downward

% ONE LEG MOTION 
% generate the cyclic motion for each leg as a sequence of backward motion
% along x, then rise up motion, move forward along x and limb down.
% making a cyclic motion with 1/4  of period as lift phase while 3/4 as
% stance, as a crawling gate

xf = 50; xb = -xf; 
y = 50; 
zu = 20; zd = 50; 
path = [xf y zd; xb y zd; xb y zu; xf y zu; xf y zd] * 1e-3; % limb path way points
p = mstraj(path, [], [0,3,0.25, 0.5, 0.25]', path(1,:), 0.01, 0); % generate the sequence of points
qcycle = leg.ikine(transl(p),'mask',[1 1 1 0 0 0]); 


figure('Name', 'Single Limb motion')
subplot(1,2,1)
hold on
for i=1:size(path,1)
    plot3(path(i,1),path(i,2),path(i,3), 'bo', 'MarkerSize', 5, 'MarkerFaceColor', 'b');
    if i== 1
        text(path(i,1),path(i,2),path(i,3)-0.001, num2str(i), 'FontSize', 10);
    else
        text(path(i,1),path(i,2),path(i,3)+0.001, num2str(i), 'FontSize', 10);
    end
end
for i=1:size(path,1)-1
    quiver3(path(i,1), path(i,2), path(i,3),path(i+1,1)-path(i,1), path(i+1,2)-path(i,2), path(i+1,3)-path(i,3), 'b', 'MaxHeadSize',1);
end
set(gca, 'Zdir', 'reverse') %set z axis downward
camproj('perspective');
xlabel('x')
ylabel('y')
zlabel('z')
grid on

subplot(1,2,2)
set(gca, 'Zdir', 'reverse') %set z axis downward
%leg.plot(qcycle, 'loop', 'perspective', 'nobase', 'notiles', 'noshadow')

% FOUR LEGS MOTION 
% define robot structure 
W = 0.1; 
L = 0.2; 

legs(1) = SerialLink(leg, 'name', 'leg1');
legs(2) = SerialLink(leg, 'name', 'leg2', 'base', transl(-L,0,0));
legs(3) = SerialLink(leg, 'name', 'leg3', 'base', transl(-L,-W,0)*trotz(pi*180/pi));
legs(4) = SerialLink(leg, 'name', 'leg4', 'base', transl(0,-W,0)*trotz(pi*180/pi));

figure('Name', 'Gate Generation')
clf; axis([-0.3 0.1 -0.2 0.2 -0.15 0.05]); 
set(gca,'Zdir', 'reverse');
hold on
% draw the robot's body
patch([0 -L -L 0], [0 0 -W -W], [0 0 0 0], ...
    'FaceColor', 'r', 'FaceAlpha', 0.5)
k = 1; 
while 1
    legs(1).plot( gait(qcycle, k, 0, 0), 'perspective', 'notiles', 'nobase', 'noshadow');
    legs(2).plot( gait(qcycle, k, 100, 0), 'perspective', 'notiles', 'nobase', 'noshadow');
    legs(3).plot( gait(qcycle, k, 200, 1), 'perspective', 'notiles', 'nobase', 'noshadow');
    legs(4).plot( gait(qcycle, k, 300, 1), 'perspective', 'notiles', 'nobase', 'noshadow');
    drawnow
    k = k+1;
end

%% ADDITIONAL USEFULL INFO: 
% see section 7.8: The plot method: 
% using SerialLink.teach(), a joint interface is open to control the pose
% in real time
