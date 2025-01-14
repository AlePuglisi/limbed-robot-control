% This function update the robot configuration and base homogeneus
% transformation, accordingly to the desired base translation 
% INPUT:
% - ROBOT    = array containing each limb as SerialLink object
% - T_base   = 4x4 Homomgeneus Transformation, base wrt fixed frame
% - q        = current limbs configuration as [N_limb x N_joint] matrix
% - x_motion = base translation on x direction, in base coordinates
% - y_motion = base translation on x direction, in base coordinates
% - z_motion = base translation on x direction, in base coordinates
% OUTPUT: 
% - q_new = [N_limb x N_joint] matrix of new limbs configuration to achieve desired translation
% - T_base    = New 4x4 Homomgeneus Transformation, base wrt fixed frame,
%               after translation

function [q_new, T_base] = translate_base(ROBOT, T_base_in,  q_in, x_motion, y_motion, z_motion)
    N_limb = length(ROBOT);
    % check which limbs are in contact
    contacts = check_contact_limbs(ROBOT);

    t_base = transl(x_motion, y_motion, z_motion); % define end desired translation 
    T_base = T_base_in * t_base;

    for i = 1:N_limb
        if contacts(i) == 1 % only contacts limb move the base
            t0 = (ROBOT(i).base.R)'*T_base_in(1:3,1:3)*t_base(1:3,4);
            ROBOT(i).base = ROBOT(i).base.T * transl(t0(1), t0(2), t0(3));

            T_tool = ROBOT(i).fkine(q_in(i,:));
            T_tool_base = (T_tool.T)^-1*T_base_in; 
            t_tool = T_tool_base(1:3,1:3)*-t_base(1:3,4);
            T_0_tool = T_tool.T*transl(t_tool(1), t_tool(2), t_tool(3));
            q_new(i,:) = ROBOT(i).ikine(T_0_tool, 'q0', q_in(i,:), 'mask', [1 1 1 0 0 0]);
        elseif contacts(i) == 0 % limbs not in contact move with the base
            t0 = (ROBOT(i).base.R)'*T_base_in(1:3,1:3)*t_base(1:3,4);
            ROBOT(i).base = ROBOT(i).base.T * transl(t0(1), t0(2), t0(3));
            q_new(i,:) = q_in(i,:);
        end
    end

end


