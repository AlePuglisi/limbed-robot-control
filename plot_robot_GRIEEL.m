% This function plot the ROBOT Limbs in contact state with configuration q
% INPUT: 
% - ROBOT = array of SerialLink object, each one describing one limb
% - q     = current limbs configuration, as an [N_limb x N_joint] matrix
function h_ee = plot_robot_GRIEEL(ROBOT, q, Mode, h_ee)
    N_limb = length(ROBOT);
    for i = 1:N_limb
           ROBOT(i).plot(q(i,:),'workspace', [-0.8 0.8 -0.8 0.8 -0.1 0.6], 'noshadow','nobase', 'notiles','noname', 'scale', 0.6); 
           T_world_ee = ROBOT(i).fkine(q(i,:));
           h_ee{i} = plot_EE(h_ee{i}, T_world_ee, Mode(i));
    end
    
    % Set axis limits manually to ensure the entire robot is visible
    xlim([-0.8 0.8]);  % Set x-axis limits
    ylim([-0.8 0.8]);  % Set y-axis limits
    zlim([ -0.1 0.6]);  % Set z-axis limits
    
    % Set equal aspect ratio to avoid distortion
    % axis equal; 
end