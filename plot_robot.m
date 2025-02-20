% This function plot the ROBOT Limbs in contact state with configuration q
% INPUT: 
% - ROBOT = array of SerialLink object, each one describing one limb
% - q     = current limbs configuration, as an [N_limb x N_joint] matrix
function plot_robot(ROBOT, q)
    N_limb = length(ROBOT);
    for i = 1:N_limb
           ROBOT(i).plot(q(i,:),'workspace', [-0.8 0.8 -0.8 0.8 -0.1 0.6], 'noshadow','nobase', 'notiles', 'scale', 0.6); 
    end
    
    % Set axis limits manually to ensure the entire robot is visible
    xlim([-0.8 0.8]);  % Set x-axis limits
    ylim([-0.8 0.8]);  % Set y-axis limits
    zlim([ -0.1 0.6]);  % Set z-axis limits
    
    % Set equal aspect ratio to avoid distortion
    % axis equal; 
    title('Robot DH Model');
end