% This function return a "mask array" with the limb in contact 
% INPUT: 
% - ROBOT = array of SerialLink object, each one describing one limb
% OTUPUT:
% - contact_mask = logical row vector of N_limb length, which has 1 in
%                  position of limbs in contact state, 0 otherwise
function contact_mask = check_contact_limbs(ROBOT)
    N_limb = length(ROBOT);
    contact_mask = zeros(1,N_limb);
    for i=1:N_limb
        if contains(ROBOT(i).name, 'contact')
            contact_mask(i) = 1; % this limb is in contact
        end
    end  
end