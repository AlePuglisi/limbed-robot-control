% This function compute the manipulability ellipsoid for the limb-end,
% considering the ability to manipulate the space as an "arm" type robot
% INPUT: 
% - ROBOT           = array containing each limb as SerialLink object
% - q               = current robot configuration, [N_limb x N_joint] matrix (each row
%                     correspond to one limb configuration)
% - limbs_mask      = mask array of which limbs ellipsoid to show 
% - h_ellipses_in   = previous handle to ellipsoids plot graphic, as [1xN] cell

% OUTPUT: 
% (being N the sum(limb_mask), limbs to consider)
% - E_limbs     = [Nx6x6] 3D matrix of Ellipsoids cores 
% - h_ellipses  = New handle to ellipsoids plot graphic, as [1xN] cell

function [E_limbs, h_ellipses] = limb_ellipsoids_general_scaledv2(ROBOT, q_in, limbs_mask, h_ellipses_in, Q_lim)
   % Clear ellipse visualization
   N_limb = length(ROBOT);
   contacts = check_contact_limbs(ROBOT); 
   names = [];
   color = [];
   for i=1:N_limb
    delete(h_ellipses_in{i});
    names = [names; strcat('Limb', num2str(i))];
    if contacts(i) == 1
        color = [color; 'b'];
    elseif contacts(i) == 0
        color = [color; 'g'];
    end
   end

    q_boundary = [];
    Q_boundary = [];

    for i=1:N_limb
        for j=1:(ROBOT(i).n-1)
            q_lim_half = (ROBOT(i).qlim(j,2) + ROBOT(i).qlim(j,1))/2;
            if(q_in(i,j) >= q_lim_half)
                q_boundary(i,j) = (ROBOT(i).qlim(j,2) - q_in(i,j))/(ROBOT(i).qlim(j,2) - ROBOT(i).qlim(j,1));
            else 
                q_boundary(i,j) = abs(q_in(i,j) - ROBOT(i).qlim(j,1))/(ROBOT(i).qlim(j,2) - ROBOT(i).qlim(j,1));
            end          
        end
        q_boundary(i,7) = 1;
    end
    Q_boundary = diag([q_boundary(1,:), q_boundary(2,:), q_boundary(3,:), q_boundary(4,:)])


   % use mask to plot ellipsoid
   disp("")
   for i=1:N_limb
       q_lim = Q_lim((i-1)*ROBOT(i).n+1:i*ROBOT(i).n, (i-1)*ROBOT(i).n+1:i*ROBOT(i).n);
       q_scale = diag(q_boundary(i,:));
       if limbs_mask(i) == 1
           J(:,:,i) = ROBOT(i).jacob0(q_in(i,:));
           J_scaled(:,:,i) = J(:,:,i)*q_lim*q_scale;
           E_limbs(:,:,i) = J_scaled(:,:,i)*J_scaled(:,:,i)';
           Et(:,:,i) = E_limbs(1:3,1:3,i);
           Er(:,:,i) = E_limbs(4:6,4:6,i);
           t_ee = ROBOT(i).fkine(q_in(i,:)).t ;        
           %h_ellipses{i} = plot_ellipse(Et(:,:,i)*0.0625, t_ee', color(i), 'alpha', 0.4);
           h_ellipses{i} = plotEllipsoidLines(56*Et(:,:,i)^-1, t_ee', color(i));
           m = sqrt(det(E_limbs(:,:,i)));
           mt = sqrt(det(Et(:,:,i)));
           str = strcat("Lim ",names(i,:) ,"  Translational Manipulability mt = ", num2str(mt));
           disp(str);
       elseif limbs_mask(i) == 0
           E_limbs(:,:,i) = zeros(6,6);
           h_ellipses{i} = plot3(0, 0, 0);
       end
   end
   disp(" ")
end
