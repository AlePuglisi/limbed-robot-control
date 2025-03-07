function [q_new, T_base, T_limb_root,r_base, h_root, h_base, h_base_poly, h_support, h_CoM] = GRIEEL_transform(ROBOT, Mode, W,  L,  q, T_base_in,T_root_in, r_base_in, h_root_in, h_base_in, h_base_poly_in, h_support_in, h_CoM_in, Q_lim)

N_contact = sum(check_contact_limbs(ROBOT));
N_limb = length(ROBOT);
if(N_contact < N_limb)
    disp("Not all Limbs in contact, Cannot transoform")
        q_new = q;
        T_base = T_base_in;
        T_limb_root = T_root_in;
        r_base = r_base_in;
        h_root =  h_root_in;
        h_base = h_base_in;
        h_base_poly =  h_base_poly_in;
        h_support = h_support_in;
        h_CoM = h_CoM_in;
        return
end

limb_seqence = [1, 2, 3, 4];  % LF, LH, RH, RF
q_config_G = [0, 0, 0];
q_config_W = [pi, pi/2, 0];
q_config = [];
if Mode == "G"
    current_mode = "W";
    q_config = q_config_G;
elseif Mode == "W"
    current_mode = "G";
    q_config = q_config_W;
else 
    disp(strcat("Invalid Mode: ", Mode, " Use Gripper ('G') or Wheel ('W')"))
    q_new = q;
    T_base = T_base_in;
    T_limb_root = T_root_in;
    r_base = r_base_in;
    h_root =  h_root_in;
    h_base = h_base_in;
    h_base_poly =  h_base_poly_in;
    h_support = h_support_in;
    h_CoM = h_CoM_in;
    return
end

disp(strcat("Start Mode Transformation to : ", Mode))
limb_names = ['LF'; 'LH'; 'RH'; 'RF'];
disp(strcat("With sequence: ", limb_names(limb_seqence(1),:) , "  -> ", limb_names(limb_seqence(2),:) , "  -> ", limb_names(limb_seqence(3),:) , "  -> ", limb_names(limb_seqence(4),:)  ))
disp("----------------------------------------------------")
h_support = h_support_in;

T_base = T_base_in;
h_root = h_root_in; 
h_base = h_base_in;
h_base_poly = h_base_poly_in; 
h_CoM = h_CoM_in;
q_new = q; 

Whole_Mode = [current_mode; current_mode; current_mode; current_mode];


close all
figure('Name','Transformation Start (Initialization)')
hold on 
for i=1:N_limb
    h_ee{i} = plot3(0,0,0);
end
h_ee = plot_robot_GRIEEL(ROBOT, q_new, Whole_Mode, h_ee);
[T_limb_root,r_base, h_root, h_base, h_base_poly, h_support, h_CoM] = update_frames(ROBOT,q_new, T_base, W, L, h_root, h_base, h_base_poly, h_support, h_CoM);

disp("MANIPULABILITY state:")
%%% BASE MANIPULABILITY ELLIPSOID  %%%
% Compute Grasp matrix and then Ellipsoid core
grasp_matrix = compute_grasp_matrix(r_base);
[E_base, Ja] = compute_base_ellipsoid_scaled(ROBOT, q_new, grasp_matrix, T_base, Q_lim);
% Plot ellipsoid, in the base frame
% h_base_ellipse = plot_ellipse(E_base(1:3,1:3),[T_base0(1,4), T_base0(2,4), T_base0(3,4)], 'r', 'alpha', 0.6);
 h_base_ellipse = plotEllipsoidLines(9*E_base(1:3,1:3)^-1,[T_base(1,4), T_base(2,4), T_base(3,4)], 'r');

%%% LIMB MANIPULABILITY ELLIPSOID %%%
% Initialization 
limbs_mask = [1 1 1 1]; % Visualize all limbs ellipsoid
% Initialize graphical element 
for i=1:sum(limbs_mask)
    h_limb_ellipses0{i} = plot_ellipse(eye(3));
end
[E_limbs, h_limb_ellipses] = limb_ellipsoids_general_scaled(ROBOT, q_new, limbs_mask, h_limb_ellipses0, Q_lim);
disp("----------------------------------------------------")
clear h_limb_ellipses0

pause()
close all


for i = limb_seqence

    current_limb_name = limb_names(limb_seqence(i),:);

    disp(strcat("Start Limb: ", limb_names(limb_seqence(i),:) , " Trasformation"));

    figure('Name',strcat('Transformation Limb ', current_limb_name))
    hold on 
    h_ee = plot_robot_GRIEEL(ROBOT, q_new, Whole_Mode, h_ee);
    [T_limb_root,r_base, h_root, h_base, h_base_poly, h_support, h_CoM] = update_frames(ROBOT,q_new, T_base, W, L, h_root, h_base, h_base_poly, h_support, h_CoM);
   
    contact_mask = check_contact_limbs(ROBOT);
    normalized = 1;
    SSM_index = SSM(ROBOT, q_new, contact_mask, T_base, 0);
    SSM_normalized = SSM(ROBOT, q_new, contact_mask, T_base, 1);
    disp(strcat('Current SSM: ', num2str(SSM_index)));
  
    dim = [0.3 0.45 0.4 0.3];
   % str = {strcat('\\bfSSM [m]: \\rm', num2str(SSM_index)),strcat('\\bfSSM Normalized [%]:\\rm ', num2str(SSM_normalized))};
   str = {sprintf('\\bfSSM [m]:\\rm %.2f', SSM_index), 
       sprintf('\\bfSSM Normalized [%%]:\\rm %.2f', SSM_normalized)};
    a = annotation('textbox',dim,'String',str,'FontSize', 20, 'FitBoxToText','on', 'Interpreter', 'tex');

    %%% BASE MANIPULABILITY ELLIPSOID  %%%
    % Compute Grasp matrix and then Ellipsoid core
    grasp_matrix = compute_grasp_matrix(r_base);
    [E_base, Ja] = compute_base_ellipsoid_scaled(ROBOT, q_new, grasp_matrix, T_base, Q_lim);
    % Plot ellipsoid, in the base frame
    % h_base_ellipse = plot_ellipse(E_base(1:3,1:3),[T_base0(1,4), T_base0(2,4), T_base0(3,4)], 'r', 'alpha', 0.6);
     h_base_ellipse = plotEllipsoidLines(9*E_base(1:3,1:3)^-1,[T_base(1,4), T_base(2,4), T_base(3,4)], 'r');
    
    %%% LIMB MANIPULABILITY ELLIPSOID %%%
    % Initialization 
    limbs_mask = [1 1 1 1]; % Visualize all limbs ellipsoid
    % Initialize graphical element 
    [E_limbs, h_limb_ellipses] = limb_ellipsoids_general_scaled(ROBOT, q_new, limbs_mask, h_limb_ellipses, Q_lim);

    contact_points = zeros(4,3);
    for c =1:N_limb
        contact_points(c,1) = h_support.XData(c);
        contact_points(c,2) = h_support.YData(c);
        contact_points(c,3) = h_support.ZData(c);
    end
    next_support = contact_points;
    next_support(i,:) = [];
    h_next_support = plot3([next_support(:,1)', next_support(1,1)],[next_support(:,2)', next_support(1,2)],[next_support(:,3)', next_support(1,3)], 'LineStyle', ' - - ', 'Color', 'b','LineWidth', 1.5);
    
    mid_next_support = [];
    mid_next_support(1) = sum(next_support(:,1) - T_base(1,4))/3;
    mid_next_support(2) = sum(next_support(:,2) - T_base(2,4))/3;
    
    mid_edge = [];
    if i == 1 | i == 3
        mid_edge(1) = (contact_points(2,1) + contact_points(4,1))/2  - T_base(1,4) ;
        mid_edge(2) = (contact_points(2,2) + contact_points(4,2))/2  - T_base(2,4);
    elseif i==2 | i==4
        mid_edge(1) = (contact_points(1,1) + contact_points(3,1))/2  - T_base(1,4) ;
        mid_edge(2) = (contact_points(1,2) + contact_points(3,2))/2  - T_base(2,4) ;
    end

    disp(strcat("Next support Polygon midpoint:  x = ", num2str(mid_next_support(1)), " | y = ",  num2str(mid_next_support(2))));
    disp(strcat("Next support Polygon Edge midpoint:  x = ", num2str(mid_edge(1)), " | y = ",  num2str(mid_edge(2))));
    
    base_x = mid_edge(1) + (mid_next_support(1) - mid_edge(1))/3;
    base_y = mid_edge(2) + (mid_next_support(2) - mid_edge(2))/3;
    
    base = [base_x, base_y, 0];
    base = T_base(1:3,1:3)*base';
    disp(" ")
   disp("press ENTER for base motion")
   disp(" ")
    pause()

    disp(strcat("Base Translation Task"));
    [q_new, T_base] = translate_base(ROBOT, T_base,  q_new, base(1), base(2), 0);
    h_ee = plot_robot_GRIEEL(ROBOT, q_new, Whole_Mode, h_ee);
    [T_limb_root,r_base, h_root, h_base, h_base_poly, h_support, h_CoM] = update_frames(ROBOT,q_new, T_base, W, L, h_root, h_base, h_base_poly, h_support, h_CoM);
     
    contact_mask = check_contact_limbs(ROBOT);
    SSM_index = SSM(ROBOT, q_new, contact_mask, T_base, 0);
    SSM_normalized = SSM(ROBOT, q_new, contact_mask, T_base, 1);
    disp(strcat('Current SSM: ', num2str(SSM_index)));
  

   % str = {strcat('\\bfSSM [m]: \\rm', num2str(SSM_index)),strcat('\\bfSSM Normalized [%]:\\rm ', num2str(SSM_normalized))};
   str = {sprintf('\\bfSSM [m]:\\rm %.2f', SSM_index), 
       sprintf('\\bfSSM Normalized [%%]:\\rm %.2f', SSM_normalized)};
    delete(a);
    a = annotation('textbox',dim,'String',str,'FontSize', 20, 'FitBoxToText','on', 'Interpreter', 'tex');


    disp("MANIPULABILITY state:")
    %%% BASE MANIPULABILITY ELLIPSOID %%% 
    % Compute Grasp matrix and then Ellipsoid core
    grasp_matrix = compute_grasp_matrix(r_base);
    [E_base, Ja] = compute_base_ellipsoid_scaled(ROBOT, q_new, grasp_matrix, T_base, Q_lim);
    % Plot ellipsoid, in the base frame
    delete(h_base_ellipse);
    % h_base_ellipse = plot_ellipse(E_base(1:3,1:3),[T_base(1,4), T_base(2,4), T_base(3,4)], 'r', 'alpha', 0.6);
    h_base_ellipse = plotEllipsoidLines(9*E_base(1:3,1:3)^-1,[T_base(1,4), T_base(2,4), T_base(3,4)], 'r');
    
    %%% LIMB MANIPULABILITY ELLIPSOID %%%
    % Initialization 
    limbs_mask = [1 1 1 1]; % Visualize limbs ellipsoid
    [E_limbs, h_limb_ellipses] = limb_ellipsoids_general_scaled(ROBOT, q_new, limbs_mask, h_limb_ellipses, Q_lim);


    delete(h_next_support);
    for c =1:N_limb
        contact_points(c,1) = h_support.XData(c);
        contact_points(c,2) = h_support.YData(c);
        contact_points(c,3) = h_support.ZData(c);
    end
    next_support = contact_points;
    next_support(i,:) = [];
    h_next_support = plot3([next_support(:,1)', next_support(1,1)],[next_support(:,2)', next_support(1,2)],[next_support(:,3)', next_support(1,3)], 'LineStyle', ' - - ',  'Color', 'b', 'LineWidth', 1.5);
    disp(" ")
    disp(strcat("press ENTER for Limb ",  current_limb_name , " Raise"))
    disp(" ")
    pause()

    delete(h_next_support);
    z_up = 0.2;
    i_rise = i; % 1=LF, 2=LH, 3=RH, 4=RF
    limb_names = ["LF*"; "LH*"; "RH*"; "RF*"];
    ROBOT(i_rise).name = limb_names(i_rise, :);

    close all
    figure('Name',strcat('Limb ', current_limb_name,  ' Up'))
    hold on 

    disp(strcat("Limb: ", limb_names(limb_seqence(i),:) , " Raise Task"));
    q_new = move_limbv2(ROBOT, q_new, i_rise, 0, 0, z_up, Mode);
    q_new(i,4) = q_new(i,6) + q_new(i,4);
    q_new(i, 5:7) = q_config;
    Whole_Mode(i) = Mode;
    h_ee = plot_robot_GRIEEL(ROBOT, q_new, Whole_Mode, h_ee);
    [T_limb_root,r_base, h_root, h_base, h_base_poly, h_support, h_CoM] = update_frames(ROBOT,q_new, T_base, W, L, h_root, h_base, h_base_poly, h_support, h_CoM);
    
    contact_mask = check_contact_limbs(ROBOT);
    SSM_index = SSM(ROBOT, q_new, contact_mask, T_base, 0);
    SSM_normalized = SSM(ROBOT, q_new, contact_mask, T_base, 1);
    disp(strcat('Current SSM: ', num2str(SSM_index)));
  
   % str = {strcat('\\bfSSM [m]: \\rm', num2str(SSM_index)),strcat('\\bfSSM Normalized [%]:\\rm ', num2str(SSM_normalized))};
   str = {sprintf('\\bfSSM [m]:\\rm %.2f', SSM_index), 
       sprintf('\\bfSSM Normalized [%%]:\\rm %.2f', SSM_normalized)};
   delete(a);
    a = annotation('textbox',dim,'String',str,'FontSize', 20, 'FitBoxToText','on', 'Interpreter', 'tex');


    disp("MANIPULABILITY state:")
    %%% BASE MANIPULABILITY ELLIPSOID  %%%
    % Compute Grasp matrix and then Ellipsoid core
    grasp_matrix = compute_grasp_matrix(r_base);
    [E_base, Ja] = compute_base_ellipsoid_scaled(ROBOT, q_new, grasp_matrix, T_base, Q_lim);
    % Plot ellipsoid, in the base frame
    delete(h_base_ellipse);
    % h_base_ellipse = plot_ellipse(E_base(1:3,1:3),[T_base(1,4), T_base(2,4), T_base(3,4)], 'r', 'alpha', 0.6);
    h_base_ellipse = plotEllipsoidLines(9*E_base(1:3,1:3)^-1,[T_base(1,4), T_base(2,4), T_base(3,4)], 'r');
    
    %%% LIMB MANIPULABILITY ELLIPSOID %%%
    % Initialization 
    limbs_mask = [1 1 1 1]; % Visualize limbs ellipsoid
    [E_limbs, h_limb_ellipses] = limb_ellipsoids_general_scaled(ROBOT, q_new, limbs_mask, h_limb_ellipses, Q_lim);
    

     disp("press ENTER for limb down")
     pause( )   

    z_down = -0.2;
    i_rise = i; % 1=LF, 2=LH, 3=RH, 4=RF
    limb_names = ["LF*_contact"; "LH*_contact"; "RH*_contact"; "RF*_contact"];
    ROBOT(i_rise).name = limb_names(i_rise, :);
    
    close all
    figure('Name',strcat('Limb ', current_limb_name,  ' Down'))
    hold on 
       
    disp(strcat("Limb: ", limb_names(limb_seqence(i),:) , " Down Task"));
    q_new = move_limbv2(ROBOT, q_new, i_rise, 0, 0, z_down, 'Down');
    q_new(i, 5:7) = q_config;
    q_new(i,4) = 0.0;
    R_EE = ROBOT(i).fkine(q_new(i,:)).R;
    theta_pitch = atan2(-R_EE(3,1), sqrt(R_EE(1,1)^2 + R_EE(2,1)^2));
    q_new(i,4) = -theta_pitch;

    h_ee = plot_robot_GRIEEL(ROBOT, q_new, Whole_Mode, h_ee);
    
    [T_limb_root,r_base, h_root, h_base, h_base_poly, h_support, h_CoM] = update_frames(ROBOT,q_new, T_base, W, L, h_root, h_base, h_base_poly, h_support, h_CoM);
    
    %%% BASE MANIPULABILITY ELLIPSOID  %%%
    disp("MANIPULABILITY state:")
    % Compute Grasp matrix and then Ellipsoid core
    grasp_matrix = compute_grasp_matrix(r_base);
    [E_base, Ja] = compute_base_ellipsoid_scaled(ROBOT, q_new, grasp_matrix, T_base, Q_lim);
    % Plot ellipsoid, in the base frame
    delete(h_base_ellipse);
    % h_base_ellipse = plot_ellipse(E_base(1:3,1:3),[T_base(1,4), T_base(2,4), T_base(3,4)], 'r', 'alpha', 0.6);
    h_base_ellipse = plotEllipsoidLines(9*E_base(1:3,1:3)^-1,[T_base(1,4), T_base(2,4), T_base(3,4)], 'r');
    
    %%% LIMB MANIPULABILITY ELLIPSOID %%%
    % Initialization 
    limbs_mask = [1 1 1 1]; % Visualize limbs ellipsoid
    [E_limbs, h_limb_ellipses] = limb_ellipsoids_general_scaled(ROBOT, q_new, limbs_mask, h_limb_ellipses, Q_lim);
       
    contact_mask = check_contact_limbs(ROBOT);
    SSM_index = SSM(ROBOT, q_new, contact_mask, T_base, 0);
    SSM_normalized = SSM(ROBOT, q_new, contact_mask, T_base, 1);
    disp(strcat('Current SSM: ', num2str(SSM_index)));
  
   % str = {strcat('\\bfSSM [m]: \\rm', num2str(SSM_index)),strcat('\\bfSSM Normalized [%]:\\rm ', num2str(SSM_normalized))};
   str = {sprintf('\\bfSSM [m]:\\rm %.2f', SSM_index), 
       sprintf('\\bfSSM Normalized [%%]:\\rm %.2f', SSM_normalized)};
   delete(a);
    a = annotation('textbox',dim,'String',str,'FontSize', 20, 'FitBoxToText','on', 'Interpreter', 'tex');
    
    disp("press ENTER for next step")

     pause()
     close all

end

disp("------------------------------- TRANSFORMTION SEQUENCE COMPLETE -------------------------------")
%% FINAL BASE CENTER
figure('Name','Final Base Centering')
hold on 
h_ee = plot_robot_GRIEEL(ROBOT, q_new, Whole_Mode, h_ee);
[T_limb_root,r_base, h_root, h_base, h_base_poly, h_support, h_CoM] = update_frames(ROBOT,q_new, T_base, W, L, h_root, h_base, h_base_poly, h_support, h_CoM);

%%% BASE MANIPULABILITY ELLIPSOID  %%%
disp("MANIPULABILITY state:")
% Compute Grasp matrix and then Ellipsoid core
grasp_matrix = compute_grasp_matrix(r_base);
[E_base, Ja] = compute_base_ellipsoid_scaled(ROBOT, q_new, grasp_matrix, T_base, Q_lim);
% Plot ellipsoid, in the base frame
delete(h_base_ellipse);
% h_base_ellipse = plot_ellipse(E_base(1:3,1:3),[T_base(1,4), T_base(2,4), T_base(3,4)], 'r', 'alpha', 0.6);
h_base_ellipse = plotEllipsoidLines(9*E_base(1:3,1:3)^-1,[T_base(1,4), T_base(2,4), T_base(3,4)], 'r');
   
contact_mask = check_contact_limbs(ROBOT);
SSM_index = SSM(ROBOT, q_new, contact_mask, T_base, normalized);
disp(strcat('Current SSM: ', num2str(SSM_index)));

str = {sprintf('\\bfSSM [m]:\\rm %.2f', SSM_index), 
   sprintf('\\bfSSM Normalized [%%]:\\rm %.2f', SSM_normalized)};
delete(a);
a = annotation('textbox',dim,'String',str,'FontSize', 20, 'FitBoxToText','on', 'Interpreter', 'tex');



%%% LIMB MANIPULABILITY ELLIPSOID %%%
% Initialization 
limbs_mask = [1 1 1 1]; % Visualize limbs ellipsoid
[E_limbs, h_limb_ellipses] = limb_ellipsoids_general_scaled(ROBOT, q_new, limbs_mask, h_limb_ellipses, Q_lim);
disp("press ENTER for base Centering")
pause()

for c =1:N_limb
       contact_points(c,1) = h_support.XData(c);
       contact_points(c,2) = h_support.YData(c);
       contact_points(c,3) = h_support.ZData(c);
end
mid_support = [];
mid_support(1) = sum(contact_points(:,1) - T_base(1,4))/4;
mid_support(2) = sum(contact_points(:,2) - T_base(2,4))/4;

[q_new, T_base] = translate_base(ROBOT, T_base,  q_new, mid_support(1), mid_support(2), 0);

for i=1:N_limb
    q_new(i, 5:7) = q_config;
    q_new(i,4) = 0.0;
    R_EE = ROBOT(i).fkine(q_new(i,:)).R;
    theta_pitch = atan2(-R_EE(3,1), sqrt(R_EE(1,1)^2 + R_EE(2,1)^2));
    q_new(i,4) = -theta_pitch;
end

h_ee = plot_robot_GRIEEL(ROBOT, q_new, Whole_Mode, h_ee);
[T_limb_root,r_base, h_root, h_base, h_base_poly, h_support, h_CoM] = update_frames(ROBOT,q_new, T_base, W, L, h_root, h_base, h_base_poly, h_support, h_CoM);
%%% BASE MANIPULABILITY ELLIPSOID  %%%
disp("MANIPULABILITY state:")
% Compute Grasp matrix and then Ellipsoid core
grasp_matrix = compute_grasp_matrix(r_base);
[E_base, Ja] = compute_base_ellipsoid_scaled(ROBOT, q_new, grasp_matrix, T_base, Q_lim);
% Plot ellipsoid, in the base frame
delete(h_base_ellipse);
% h_base_ellipse = plot_ellipse(E_base(1:3,1:3),[T_base(1,4), T_base(2,4), T_base(3,4)], 'r', 'alpha', 0.6);
h_base_ellipse = plotEllipsoidLines(9*E_base(1:3,1:3)^-1,[T_base(1,4), T_base(2,4), T_base(3,4)], 'r');

%%% LIMB MANIPULABILITY ELLIPSOID %%%
% Initialization 
limbs_mask = [1 1 1 1]; % Visualize limbs ellipsoid
[E_limbs, h_limb_ellipses] = limb_ellipsoids_general_scaled(ROBOT, q_new, limbs_mask, h_limb_ellipses, Q_lim);

contact_mask = check_contact_limbs(ROBOT);
SSM_index = SSM(ROBOT, q_new, contact_mask, T_base, normalized);
disp(strcat('Current SSM: ', num2str(SSM_index)));

str = {sprintf('\\bfSSM [m]:\\rm %.2f', SSM_index), 
   sprintf('\\bfSSM Normalized [%%]:\\rm %.2f', SSM_normalized)};
delete(a);
a = annotation('textbox',dim,'String',str,'FontSize', 20, 'FitBoxToText','on', 'Interpreter', 'tex');




%% DRIVING MODE 

disp("press ENTER for DRIVING mode")
pause()


q_new(1,5) = q_new(1,5) - pi/4; 
q_new(3,5) = q_new(3,5) -  pi/4;
q_new(2,5) = q_new(2,5)  + pi/4;
q_new(4,5) =  q_new(4,5) + pi/4; 

h_ee = plot_robot_GRIEEL(ROBOT, q_new, Whole_Mode, h_ee);
[T_limb_root,r_base, h_root, h_base, h_base_poly, h_support, h_CoM] = update_frames(ROBOT,q_new, T_base, W, L, h_root, h_base, h_base_poly, h_support, h_CoM);
%%% BASE MANIPULABILITY ELLIPSOID  %%%
disp("MANIPULABILITY state:")
% Compute Grasp matrix and then Ellipsoid core
grasp_matrix = compute_grasp_matrix(r_base);
[E_base, Ja] = compute_base_ellipsoid_scaled(ROBOT, q_new, grasp_matrix, T_base, Q_lim);
% Plot ellipsoid, in the base frame
delete(h_base_ellipse);
% h_base_ellipse = plot_ellipse(E_base(1:3,1:3),[T_base(1,4), T_base(2,4), T_base(3,4)], 'r', 'alpha', 0.6);
h_base_ellipse = plotEllipsoidLines(9*E_base(1:3,1:3)^-1,[T_base(1,4), T_base(2,4), T_base(3,4)], 'r');

%%% LIMB MANIPULABILITY ELLIPSOID %%%
% Initialization 
limbs_mask = [1 1 1 1]; % Visualize limbs ellipsoid
[E_limbs, h_limb_ellipses] = limb_ellipsoids_general_scaled(ROBOT, q_new, limbs_mask, h_limb_ellipses, Q_lim);

contact_mask = check_contact_limbs(ROBOT);
SSM_index = SSM(ROBOT, q_new, contact_mask, T_base, normalized);
disp(strcat('Current SSM: ', num2str(SSM_index)));

str = {sprintf('\\bfSSM [m]:\\rm %.2f', SSM_index), 
   sprintf('\\bfSSM Normalized [%%]:\\rm %.2f', SSM_normalized)};
delete(a);
a = annotation('textbox',dim,'String',str,'FontSize', 20, 'FitBoxToText','on', 'Interpreter', 'tex');

