% This function updates the visualization of limb_root and base frames
% INPUT: 
% - ROBOT        = array of SerialLink object, each one describing one limb
% - q            = current limbs configuration, as an [N_limb x N_joint] matrix
% - T_base       =  4x4 Homomgeneus Transformation, base wrt fixed frame
% - W            = base width in meters
% - L            = base length in meters
% - h_root_in    = handle to root_link SE3 plot graphic, as [1x4] cell, each limb
% - h_base_in    = handle to base SE3 graphic
% - h_patch_in   = handle to base patch graphic
% - h_support_in =  handle to support polygon patch graphic
% OUTPUT: 
% - T_limb_root = 4x4 Homomgeneus Transformation, limb root wrt fixed frame
% - r_base      = matrix [N_limb x 3] containing in each row the vector of the limb
%                 "graspoing point" with the robot base (limb-base attachment)
% - h_root      = New handle to root_link SE3 plot graphic, as [1x4] cell, each limb
% - h_base      = New handle to base SE3 graphic
% - h_patch     = New handle to base patch graphic
% - h_support   = New handle to support polygon patch graphic

function [T_limb_root,r_base, h_root, h_base, h_base_poly, h_support, h_CoM] = update_frames(ROBOT, q, T_base, W, L, h_root_in, h_base_in, h_base_poly_in, h_support_in, h_CoM_in)
    % count limbs
    N_limb = length(ROBOT);
    % check which limbs are in contact
    contacts = check_contact_limbs(ROBOT);
    N_limb_contact = sum(contacts);

    % delete graphics 
    delete(h_support_in);
    for i=1:N_limb
        delete(h_root_in{i}); % limbs root frames 
    end
    delete(h_base_poly_in); % base perimeter polygon  
    delete(h_support_in);   % support polygon (N_limb_contact lines) 
    delete(h_base_in);  % base frame
    delete(h_CoM_in);   % CoM projection 
    
    % Initialize homogeneus transformation matrices
    T_coxa_limb_root = zeros(4,4,N_limb);
    T_coxa = zeros(4,4,N_limb);
    T_limb_root = zeros(4,4,N_limb);
    
    t_support = [];
    
    for i = 1:N_limb
        if contacts(i) == 1
            T_limb_root(:,:,i) = ROBOT(i).base; % when not in contact, limb root coincide with limb base
            h_root{i} = trplot(T_limb_root(:,:,i), 'rgb', 'length', 0.15, 'arrow');
            
            T = transl(-sqrt((W/2)^2+(L/2)^2), 0, 0);
            %r_base(i,:) = T_base(1:3,1:3)^-1*(T_limb_root(1:3,1:3,i)*T(1:3,4));
            r_base(i,:) = ROBOT(i).base.t';

            t_support = [t_support; ROBOT(i).fkine(q(i,:)).t'];

        elseif contacts(i) == 0
            T_limb_root(:,:,i) = ROBOT(i).base; % when not in contact, limb root coincide with limb base
            h_root{i} = trplot(T_limb_root(:,:,i), 'rgb', 'length', 0.15, 'arrow');
            % No need to compute root to base radius, because no
            % contribution to the grasp matrix is given by this limb
            r_base(i,:) = zeros(1,3);
        end
    end
    
    h_base = trplot(T_base, 'rgb', 'length', 0.15, 'arrow', 'framelabel', 'base');
    
    x_root = [];
    y_root = [];
    z_root = [];
    for i=1:N_limb
        x_root = [x_root, T_limb_root(1,4,i)];
        y_root = [y_root, T_limb_root(2,4,i)];
        z_root = [z_root, T_limb_root(3,4,i)];
    end
    x_root = [x_root, T_limb_root(1,4,1)];
    y_root = [y_root, T_limb_root(2,4,1)];
    z_root = [z_root, T_limb_root(3,4,1)];

    h_base_poly = plot3(x_root,y_root,z_root, 'Color', 'r', 'LineWidth', 1.0);
    
    % Plot support polygon
    if N_limb_contact > 1
        h_support = plot3([t_support(:,1)', t_support(1,1)],[t_support(:,2)', t_support(1,2)],[t_support(:,3)', t_support(1,3)], 'Color', 'g', 'LineWidth', 1.0);
    end

    % Plot CoM Projection
    x_com = T_base(1,4);
    y_com = T_base(2,4);
    z_com = 0;
    h_CoM = plot3(x_com, y_com, z_com,'o','Color','#FF6E00','MarkerSize',10,'MarkerFaceColor','#FF6E00');

end