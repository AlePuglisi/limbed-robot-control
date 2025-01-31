% This function compute the core 6x6 matrix of the base velocity elliposid
% INPUT:
% - ROBOT =  array containing each limb as SerialLink object
% - q     = current robot configuration, [N_limb x 6] matrix (each row
%           correspond to one limb configuration)
% - W     = grasp matrix
% OUTPUT: 
% - base_ellispoid = correspond to the core matrix E s.t the ellipsoid is
%                    descriped  by v*(E)^-1*v'<=1
% -Ja              = corresponding grasp Jacobian matrix, relating joint speed to base absolute speed
               
function [base_ellipsoid,Ja] = compute_base_ellipsoid(ROBOT, q, grasp_matrix, T_base)
    N_limb = length(ROBOT); 
    N_joint = ROBOT(1).n;
    contacts = check_contact_limbs(ROBOT);
    N_limb_contact = sum(contacts);

    J_full = zeros(N_limb_contact*6,N_limb_contact*N_joint);
    ROBOT_CONTACT = [];
    q_new = [];
    for i=1:N_limb
        if contacts(i) == 1
            ROBOT_CONTACT = [ROBOT_CONTACT, ROBOT(i)];
            q_new = [q_new; q(i,:)];
        end
    end

    for i=1:N_limb_contact
        % A change in the reference frame is needed, we use common origin jacobian  
        
        %J_full(1+(i-1)*6:6+(i-1)*6, 1+(i-1)*N_joint:N_joint+(i-1)*N_joint) = tr2jac(ROBOT_CONTACT(i).base, 'samebody')*ROBOT_CONTACT(i).jacob0(q_new(i,:));
     
        T_ee_base = (ROBOT_CONTACT(i).fkine(q_new(i,:)).T)^-1*T_base
        T_ee_0 = (ROBOT_CONTACT(i).fkine(q_new(i,:)).T)^-1; %*ROBOT_CONTACT(i).base.T;
 
        J_full(1+(i-1)*6:6+(i-1)*6, 1+(i-1)*N_joint:N_joint+(i-1)*N_joint) = tr2jac(T_ee_base)*ROBOT_CONTACT(i).jacobe(q_new(i,:));
        %J_full(1+(i-1)*6:6+(i-1)*6, 1+(i-1)*N_joint:N_joint+(i-1)*N_joint) = tr2jac(T_ee_0)*ROBOT_CONTACT(i).jacobe(q_new(i,:));
    end

    Ja = (J_full'*pinv(grasp_matrix))';
    base_ellipsoid = Ja*Ja';

    m = sqrt(det(base_ellipsoid(1:3,1:3)));
    disp("Base Translational Manipulability: " + num2str(real(m)));

end