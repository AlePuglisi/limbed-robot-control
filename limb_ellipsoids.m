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

function [E_limbs, h_ellipses] = limb_ellipsoids(ROBOT, q_in, limbs_mask, h_ellipses_in)
   % Clear ellipse visualization
   N_limb = length(ROBOT);
   contacts = check_contact_limbs(ROBOT); 
   names = ["LF"; "LH"; "RH"; "RF"];
   color = [];
   for i=1:N_limb
    delete(h_ellipses_in{i});
    if contacts(i) == 1
        color = [color; 'b'];
    elseif contacts(i) == 0
        color = [color; 'g'];
    end
   end

   % use mask to plot ellipsoid
   disp("")
   for i=1:N_limb
       if limbs_mask(i) == 1
           J(:,:,i) = ROBOT(i).jacob0(q_in(i,:));
           E_limbs(:,:,i) = J(:,:,i)*J(:,:,i)';
           Et(:,:,i) = E_limbs(1:3,1:3,i);
           Er(:,:,i) = E_limbs(4:6,4:6,i);
           t_ee = ROBOT(i).fkine(q_in(i,:)).t ;        
           %h_ellipses{i} = plot_ellipse(Et(:,:,i)*0.0625, t_ee', color(i), 'alpha', 0.4);
           h_ellipses{i} = plotEllipsoidLines(5*Et(:,:,i)^-1, t_ee', color(i));
           m = sqrt(det(E_limbs(:,:,i)));
           mt = sqrt(det(Et(:,:,i)));
           str = "Limb " + names(i,:) + " Translational Manipulability mt = " + num2str(mt);
           disp(str);
       elseif limbs_mask(i) == 0
           E_limbs(:,:,i) = zeros(6,6);
           h_ellipses{i} = plot3(0, 0, 0);
       end
   end
   disp("")
end
