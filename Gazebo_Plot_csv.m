clear 
clc
close all 

%% READ BAG csv results: 

file = "G2W";

path = "data_experiments_simulation/Harmonic_sim/";
if strcmp(file,"G2W")
    path = strcat(path, "G2W/");
elseif strcmp(file,"W2G")
    path = strcat(path, "W2G/");
elseif strcmp(file,"drive")
    path = strcat(path, "Driving_3Wheel/");
end

LF = csvread(strcat(path, "LF_joint_data.csv"),1,0);
LH = csvread(strcat(path, "LH_joint_data.csv"),1,0);
RH = csvread(strcat(path,"RH_joint_data.csv"),1,0);
RF = csvread(strcat(path,"RF_joint_data.csv"),1,0);

[numR_LF, numC_LF] = size(LF) ;
range_LF = [1, 0, numR_LF-1, numC_LF-2]; % Zero-based indexing!
LF = csvread(strcat(path, "LF_joint_data.csv"), 1, 0, range_LF);

[numR_LH, numC_LH] = size(LH); 
range_LH = [1, 0, numR_LH-1, numC_LH-2]; % Zero-based indexing!
LH = csvread(strcat(path, "LH_joint_data.csv"), 1, 0, range_LH);

[numR_RH, numC_RH] = size(RH);
range_RH = [1, 0, numR_RH-1, numC_RH-2]; % Zero-based indexing!
RH = csvread(strcat(path, "RH_joint_data.csv"), 1, 0, range_RH);

[numR_RF, numC_RF] = size(RF); 
range_RF = [1, 0, numR_RF-1, numC_RF-2]; % Zero-based indexing!
RF = csvread(strcat(path, "RF_joint_data.csv"), 1, 0, range_RF);

limbs_data = {};
limbs_data{1} = LF; 
limbs_data{2}= LH; 
limbs_data{3} = RH; 
limbs_data{4} = RF; 

%% REORDER/COLLECT DATA: 

N_joint = 7; 
N_limbs = 4; 

F2T_offset = 1.364075222;
T2E_offset = 0.2067211047;

time_steps = [];

for i=1:N_limbs
    F2T_index = 6;
    T2E_index = 8;

    limbs_data{i}(:,1) =  limbs_data{i}(:,1) - limbs_data{i}(1,1);
    limbs_data{i}(:,1) = limbs_data{i}(:,1)/1e9;

    % limbs_data{i}(:,F2T_index) =  limbs_data{i}(:,F2T_index) + F2T_offset;
    % limbs_data{i}(:,F2T_index+1) =  limbs_data{i}(:,F2T_index+1) + F2T_offset;
    % 
    % limbs_data{i}(:,T2E_index) =  limbs_data{i}(:,T2E_index) + T2E_offset;
    % limbs_data{i}(:,T2E_index+1) =  limbs_data{i}(:,T2E_index+1) + T2E_offset;
    
    time_steps(i) = size(limbs_data{i}, 1);
end

time_steps = min(time_steps);
q = [];
contact_mask = [];
for t=1:time_steps
    for i=1:N_limbs
        contact_mask(t, i) = 1;
        for j=1:N_joint
            q(i,j,t) = limbs_data{i}(t, j*3);
        end
        if q(i,2,t) < -1.2
            contact_mask(t,i) = 0; 
        end
    end
   % q(:,7,t) = zeros(N_limbs,1); % correct driving joint 
end

%% AVERAGE TRACKING ERROR
avg_errors = [];
max_errors = [];

for i=1:N_limbs
    for  j=1:N_joint-1
         joint_state_index = j*3;
         joint_ref_index =  j*3 - 1;
         avg_errors(i,j) = sum(abs(limbs_data{i}(200:end,joint_ref_index)-limbs_data{i}(200:end,joint_state_index)))/size(limbs_data{i}(200:end,joint_state_index), 1);
         max_errors(i,j) = max(abs(limbs_data{i}(200:end,joint_ref_index)-limbs_data{i}(200:end,joint_state_index)));
    end
end

%% PLOT DATA: 
limb_names = ["LF"; "LH"; "RH"; "RF"];
joint_names = ["B2C"; "C2F"; "F2T"; "T2E"; "wristH"; "wristV"; "driving"];
for i=1:N_limbs
    name = strcat("Limb ", limb_names(i), ": Joint state tracking");
    figure('Name',name)
    for  j=1:N_joint 
        subplot(2,4,j)
        joint_state_index = j*3;
        joint_ref_index =  j*3 - 1;
        plot(limbs_data{i}(:,1), limbs_data{i}(:,joint_ref_index), 'Color', '#0E6926', 'LineStyle','--', 'LineWidth',2.0);
        hold on 
        plot(limbs_data{i}(:,1), limbs_data{i}(:,joint_state_index), 'b', 'LineWidth',2.0);
        plot(limbs_data{i}(:,1), limbs_data{i}(:,joint_ref_index)-limbs_data{i}(:,joint_state_index), 'r-');
        grid on 
        xlabel('time[s]')
        if j<7
            ylabel('joint angle [rad]')
        else
            ylabel('joint speed [rad/s]')
        end 
        legend('reference','actual', 'error')
        title(joint_names(j))
         ax = gca;               % Get current axes
        ax.FontSize = 16;       % Set font size
        hold off
    end
    sgtitle(strcat("Limb ", limb_names(i), ": Joint state tracking performance"));
end

for i=1:N_limbs
    name = strcat("Limb ", limb_names(i), ": Joint Control Torque");
    figure('Name',name)
    for  j=1:N_joint 
        subplot(2,4,j)
        joint_tau_index =  j*3 + 1;
        plot(limbs_data{i}(:,1), limbs_data{i}(:,joint_tau_index), 'Color', '#0E6926', 'LineWidth',2.0);
        hold on 
        grid on 
        xlabel('time[s]')
        ylabel('joint torque [Nm]')
        axis([0,limbs_data{i}(end,1), -14, 14]);
        legend('torque')
        title(joint_names(j))
        ax = gca;               % Get current axes
        ax.FontSize = 16;       % Set font size
        hold off
    end
    sgtitle(strcat("Limb ", limb_names(i), ":  Joint Control Torque"));
end

if strcmp(file,"drive")
     
     figure('Name',"driving velocity Tracking") 
    for i=1:N_limbs
        subplot(2,4,i)
        plot(limbs_data{i}(:,1), limbs_data{i}(:,7*3), 'Color', 'b', 'LineWidth',2.0);
        hold on
        grid on 
        xlabel('time[s]')
        ylabel('joint velocity [rad/s]')
        legend('velocity')
        name = strcat("Limb ", limb_names(i), " driving joint velocity");
        title(name)
        ax = gca;               % Get current axes
        ax.FontSize = 16;       % Set font size
        axis([0,limbs_data{i}(end,1), -15, 15]);
        hold off
        subplot(2,4,i+4)
        plot(limbs_data{i}(:,1), limbs_data{i}(:,7*3+1),'Color', '#0E6926', 'LineWidth',2.0);
        hold on
        grid on 
        xlabel('time[s]')
        ylabel('driving torque [Nm]')
        legend('torque')
        name = strcat("Limb ", limb_names(i), " driving torque");
        title(name)
        ax = gca;               % Get current axes
        ax.FontSize = 16;       % Set font size
        axis([0,limbs_data{i}(end,1), -15, 15]);
        hold off
        sgtitle("Driving Velocity tracking performance");
    end
end
% Plot Robot
disp("--------------------------------------")
disp("PRESS ENTER TO START VIEW ROBOT MODEL:")
pause()

%% PLOT THE ROBOT DH MODEL 
% SWING
load('limbero_data');

Limb = limbero; 
Limb.qlim = [-2.10, 2.10; -1.95, 1.30; -0.40, 2.95; -pi/2, pi/2; -pi, pi; -0.25*pi, 0.5*pi; -inf, inf];

%% DEFINE ROBOT MODEL 
N_limb = 4; 
W = 0.25; 
L = 0.25; 
T_tool = trotx(pi/2)*troty(pi/2)*trotz(pi/2);
q0 = [0, -pi/6, pi/2+pi/6, 0, 0, 0, 0] ;
ROBOT = Robot_model(W, L, Limb, q0, [1 1 1 1], T_tool);

% PLOT CHARACTERISTICS
axis_x_l= -0.6; 
axis_x_u= 0.6;
axis_y_l= -0.6; 
axis_y_u= 0.6;
axis_z_l= -0.25; 
axis_z_u= 0.6;

%% INITIALIZE PLOTTING 
figure('Name', 'LIMBERO-GRIEEL')
hold on 
plot_robot(ROBOT, q(:,:,1));

% DEFINE  SCALING MATRIX 
q_dot_lim_i = [3.37, 3.37, 3.88, 3.88, 4.47, 4.47, 3.88 ];
q_dot_lim = [];
for i=1:N_limb
    q_dot_lim = [q_dot_lim, q_dot_lim_i];
end
Q_lim = diag(q_dot_lim);

% PLOT FRAMES 
% Initialize graphics elements
T_base0 = transl(0,0,ROBOT(1).base.t(3));

for i = 1:N_limb
    h_root0{i} = trplot(eye(4)); 
end
h_base0 = trplot(T_base0,'rgb', 'length', 0.1, 'arrow');
h_CoM0 = plot3(0,0,0);
h_base_poly0 = plot3([0 0 0 0 0],[0 0 0 0 0],[0 0 0 0 0]);
h_support0 = plot3([0 0 0 0 0],[0 0 0 0 0],[0 0 0 0 0]);

% Create graphics 
[T_limb_root,r_base, h_root, h_base, h_base_poly, h_support, h_CoM] = update_frames(ROBOT, q(:,:,1), T_base0, W, L, h_root0, h_base0, h_base_poly0, h_support0, h_CoM0);
clear h_root0 h_base0 h_CoM0 h_base_poly0 h_support0

plot_ellipsoid = 0;
if plot_ellipsoid == 1
    % BASE MANIPULABILITY ELLIPSOID 
    % Compute Grasp matrix and then Ellipsoid core
    grasp_matrix = compute_grasp_matrix(r_base);
    [E_base, Ja] = compute_base_ellipsoid_scaled(ROBOT, q(:,:,1), grasp_matrix, T_base0, Q_lim);
    % Plot ellipsoid, in the base frame
    % h_base_ellipse = plot_ellipse(E_base(1:3,1:3),[T_base0(1,4), T_base0(2,4), T_base0(3,4)], 'r', 'alpha', 0.6);
     h_base_ellipse = plotEllipsoidLines(9*E_base(1:3,1:3)^-1,[T_base0(1,4), T_base0(2,4), T_base0(3,4)], 'r');
    
    % LIMB MANIPULABILITY ELLIPSOID
    % Initialization 
    limbs_mask = [1 1 1 1]; % Visualize all limbs ellipsoid
    % Initialize graphical element 
    for i=1:sum(limbs_mask)
        h_limb_ellipses0{i} = plot_ellipse(eye(3));
    end
    [E_limbs, h_limb_ellipses] = limb_ellipsoids_general_scaled(ROBOT, q(:,:,1), limbs_mask, h_limb_ellipses0, Q_lim);
    clear h_limb_ellipses0
end

SSM_index = SSM(ROBOT, q(:,:,1), contact_mask(1,:), T_base0, 0);
SSM_normalized = SSM(ROBOT,  q(:,:,1), contact_mask(1,:), T_base0, 1);
disp(strcat('Current SSM: ', num2str(SSM_index)));

dim = [0.3 0.45 0.4 0.3];
% str = {strcat('\\bfSSM [m]: \\rm', num2str(SSM_index)),strcat('\\bfSSM Normalized [%]:\\rm ', num2str(SSM_normalized))};
str = {sprintf('\\bfSSM [m]:\\rm %.2f', SSM_index), 
sprintf('\\bfSSM Normalized [%%]:\\rm %.2f', SSM_normalized)};
a = annotation('textbox',dim,'String',str,'FontSize', 20, 'FitBoxToText','on', 'Interpreter', 'tex');

dim_time = [0.3 0.5 0.4 0.3];
str_time = {sprintf('\\bftime [s]:\\rm %.2f: \\rm %.2f',  limbs_data{3}(1,1))};
time_text = annotation('textbox',dim_time,'String',str_time,'FontSize', 15, 'FitBoxToText','on', 'Interpreter', 'tex', 'EdgeColor','w');
axis([axis_x_l axis_x_u axis_y_l axis_y_u axis_z_l axis_z_u]);

raised = [0, 0, 0, 0]; 
limb_names_contact = ["LF_{contact}"; "LH_{contact}"; "RH_{contact}"; "RF_{contact}"];

% Plot Robot
disp("--------------------------------------")
disp("PRESS ENTER TO START MOTION:")
pause()

SSM_signal = [];
SSM_signal_normalized = [];

%% START CYCLE

dt = 2;
T_tot =    limbs_data{3}(end,1);
step = ceil((time_steps/T_tot)*dt);

%%
k = 1;
for t = 1:step:time_steps
    for i=1:N_limb
        if contact_mask(t,i) == 0 && raised(i) == 0
            close all
            figure('Name', 'LIMBERO-GRIEEL')
            hold on 
            ROBOT(i).name = limb_names(i, :);
            raised(i) = 1;
        end
        if  contact_mask(t,i) == 1 && raised(i) == 1
            close all
            figure('Name', 'LIMBERO-GRIEEL')
            hold on 
            ROBOT(i).name = limb_names_contact(i, :);
            raised(i) = 0;
        end         
    end

    plot_robot(ROBOT, q(:,:,t));
    [T_limb_root,r_base, h_root, h_base, h_base_poly, h_support, h_CoM] = update_frames(ROBOT, q(:,:,t), T_base0, W, L, h_root, h_base, h_base_poly, h_support, h_CoM);
    
    if plot_ellipsoid == 1
        %%% BASE MANIPULABILITY ELLIPSOID  %%%
        % Compute Grasp matrix and then Ellipsoid core
        grasp_matrix = compute_grasp_matrix(r_base);
        [E_base, Ja] = compute_base_ellipsoid_scaled(ROBOT,  q(:,:,t), grasp_matrix, T_base0, Q_lim);
        % Plot ellipsoid, in the base frame
        delete(h_base_ellipse);
        % h_base_ellipse = plot_ellipse(E_base(1:3,1:3),[T_base(1,4), T_base(2,4), T_base(3,4)], 'r', 'alpha', 0.6);
        h_base_ellipse = plotEllipsoidLines(9*E_base(1:3,1:3)^-1,[T_base0(1,4), T_base0(2,4), T_base0(3,4)], 'r');
        
        %%% LIMB MANIPULABILITY ELLIPSOID %%%
        % Initialization 
        limbs_mask = [1 1 1 1]; % Visualize limbs ellipsoid
        [E_limbs, h_limb_ellipses] = limb_ellipsoids_general_scaled(ROBOT,  q(:,:,t), limbs_mask, h_limb_ellipses, Q_lim);
    end

    SSM_index = SSM(ROBOT, q(:,:,t), contact_mask(t,:), T_base0, 0);
    SSM_normalized = SSM(ROBOT,  q(:,:,t), contact_mask(t,:), T_base0, 1);
     str = {sprintf('\\bfSSM [m]:\\rm %.2f', SSM_index), 
           sprintf('\\bfSSM Normalized [%%]:\\rm %.2f', SSM_normalized)};
     delete(a);
     a = annotation('textbox',dim,'String',str,'FontSize', 20, 'FitBoxToText','on', 'Interpreter', 'tex');
     
     delete(time_text);
     str_time = {sprintf('\\bftime [s]:\\rm %.2f: \\rm %.2f',  limbs_data{3}(t,1))};
     time_text = annotation('textbox',dim_time,'String',str_time,'FontSize', 15, 'FitBoxToText','on', 'Interpreter', 'tex', 'EdgeColor','w');
    axis([axis_x_l axis_x_u axis_y_l axis_y_u axis_z_l axis_z_u]);

     SSM_signal(k) = SSM_index;
     SSM_signal_normalized(k) = SSM_normalized;
     k = k+1;
     pause();
end


%% SSM ANALYSIS
figure('Name', 'SSM Analysis')
times = limbs_data{1}(1:step:time_steps,1);

subplot(1,2,1)
plot(times', SSM_signal', 'LineWidth',1.5);
grid on
title('SSM')
xlabel('Time [s]')
ylabel('SSM [m]')
ax = gca;               % Get current axes
ax.FontSize = 16;       % Set font size

subplot(1,2,2)
plot(times', SSM_signal_normalized','LineWidth',1.5);
grid on 
title('SSM Normalized')
xlabel('Time [s]')
ylabel('SSM_{%}')
ax = gca;               % Get current axes
ax.FontSize = 16;       % Set font size


