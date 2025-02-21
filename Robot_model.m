
% This function return the Robot model as an array of SerialLink objects,
% each one as a limb. 
% INPUT: 
% - W            = Base width
% - L            = Base Length
% - limb         = SerialLink object to use as limb in swing mode
% - contact_mask = logic array indicating which limb is in contact and
%                  which one not

% OUTPUT:
% - Robot        = Robot model 
function Robot = Robot_model(W, L, limb, q0_contact_swing,  contact_mask, T_tool)
    N_limb = length(contact_mask);
    g = [0; 0; 9.81];             % Gravity vector

    % Homogeneus Transformations for Base to Limbroot position CONTACT MODE
%     q0_contact = zeros(1,limb.n); % zero joint configuration 
%     tx_ee_contact = limb_contact.fkine(q0_contact).t(1); % x-coordinate of limb tool = limb length
%     t = tx_ee_contact*sqrt(2)/2; % transaltion to move limb base to correct position
% 
%     T_LF_contact = transl(L/2+t, W/2+t, tool_length)*trotz(-(pi/2+pi/4)*180/pi)*limb_contact.base.T;
%     T_LH_contact =  transl(-L/2-t, W/2+t, tool_length)*trotz(-(pi/4)*180/pi)*limb_contact.base.T;
%     T_RH_contact =  transl(-L/2-t, -W/2-t, tool_length)*trotz((pi/4)*180/pi)*limb_contact.base.T;
%     T_RF_contact =  transl(L/2+t, -W/2-t, tool_length)*trotz((pi/4+pi/2)*180/pi)*limb_contact.base.T;
%     
%     T_contact(:,:,1) = T_LF_contact;
%     T_contact(:,:,2) = T_LH_contact;
%     T_contact(:,:,3) = T_RH_contact;
%     T_contact(:,:,4) = T_RF_contact;  

    % Homogeneus Transformations for Base to Limbroot position SWING MODE

    tz_ee = limb.fkine(q0_contact_swing).t(3); % z-coordinate of limb tool = limb height
    T_LF = transl(L/2, W/2, -tz_ee)*trotz(pi/4);
    T_LH =  transl(-L/2, W/2, -tz_ee)*trotz((pi/2+pi/4));
    T_RH =  transl(-L/2, -W/2, -tz_ee)*trotz((-pi/2-pi/4));
    T_RF =  transl(L/2, -W/2, -tz_ee)*trotz(-pi/4);
    T(:,:,1) = T_LF;
    T(:,:,2) = T_LH;
    T(:,:,3) = T_RH;
    T(:,:,4) = T_RF;
    
    names = ['LF*'; 'LH*'; 'RH*'; 'RF*'];
    Robot = [];
    for i=1:N_limb
         if contact_mask(i) == 0
            Robot = [Robot, SerialLink(limb,'name', names(i,:), 'gravity', g, 'base', T(:,:,i), 'tool', T_tool)];
         elseif contact_mask(i) == 1
            Robot = [Robot, SerialLink(limb,'name', strcat(names(i,:),'_{contact}'), 'gravity', g, 'base', T(:,:,i), 'tool', T_tool)];
         end
    end
    
end
