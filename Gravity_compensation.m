% This function compute the gravitational torque of a robot
% manipiulator as function of the configuration q, considering all joints
% as revolute (q = theta). 
% It is based on explicit computation of gravitational load accordingly to
% Lagrangian formulation of robotic arm dynamic model 

% it takes as input:
% a, d, alpha, offset = DH parameters of the robot model                [1 x N_link]
% r_cm                = matrix of CoM positions in each link frame      [N_link x 3] 
%                       (r_cm(i,:) = [ r_cm_x(i),r_cm_y(i), r_cm_z(i)]) 
% m                   = vector of links mass                            [N_link x 1]
% q                   = robot joint configuration                       [1 x N_joints]
% contact             = ground contact state [0: not contact, 1: contact]  

function Taug = Gravity_compensation(a, d, alpha,offset, r_cm, m, q, contact)
% syms q [length(a) 1]

N_link = length(a);

% Homogeneus transformation matrices between refrence frames (i-1)_(i)
% A_(i-1)_(i)
A_beforei_i = zeros(4,4,N_link);
for i=1:N_link
    A_beforei_i(:,:,i) = [cos(q(i)+offset(i)), -sin(q(i)+offset(i))*cos(alpha(i)), sin(q(i)+offset(i))*sin(alpha(i)), a(i)*cos(q(i)+offset(i));
                          sin(q(i)+offset(i)), cos(q(i)+offset(i))*cos(alpha(i)), -cos(q(i)+offset(i))*sin(alpha(i)), a(i)*sin(q(i)+offset(i));
                             0     ,     sin(alpha(i))      ,   cos(alpha(i))         ,    d(i)       ;
                             0     ,        0               ,       0                 ,     1         ];
end

% Homogeneus transformation between base frame 0 to frame (i)
A_0_i = zeros(4,4,N_link);
A_0_i(:,:,1) = A_beforei_i(:,:,1); % A_0_1
for i=2:N_link
     A_0_i(:,:,i) = A_0_i(:,:,i-1)*A_beforei_i(:,:,i); % A_0_i
end

% Compute CoM position in the base frame

p_cm_tilde = zeros(N_link, 4);
p_cm = zeros(N_link, 3);
for i=1:N_link
    p_cm_tilde(i,:) = A_0_i(:,:,i)*[r_cm(i,:), 1]';
    p_cm(i,:) = p_cm_tilde(i,1:3);
end

% extract links origin and z axis in base frame 
p_o = zeros(N_link, 3);
z = zeros(N_link, 3);
for i=1:N_link
    p_o(i,:) = A_0_i(1:3,4,i);
    z(i,:) = A_0_i(1:3,3,i);
end

% create the CoM jacobian matrices 
Jp_lj = zeros(3, N_link, N_link);
for j=1:N_link
    for i=1:N_link
        if i == 1
            z_0 = [0, 0, 1];
            p_o_0 = [0, 0, 0];
            Jp_lj(:,i,j) = cross(z_0,p_cm(j,:) - p_o_0);

        elseif i > 1
            if i > j
                Jp_lj(:,i,j) = zeros(3,1);
            else
                Jp_lj(:,i,j) = cross(z(i-1,:), (p_cm(j,:)-p_o(i-1,:)));
            end
        end
    end
end

if contact == 1
    g0 = [0,0, 9.81]'; % gravity
elseif contact == 0
    g0 = [0,0,-9.81]';
end

G = zeros(1,N_link);
for i=1:N_link
    G(i) = 0;
    for j=1:N_link
            G(i) = G(i) - m(j)*g0'*Jp_lj(:,i,j);
    end
end

Taug = G;

