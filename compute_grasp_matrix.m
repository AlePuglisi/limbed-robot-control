% this function compute the grasp matrix given the vector of the limbr root
% frames, in the fixed inertia frame

% INPUT: 
% - r = matrix [N_limb_contact x 3] containing in each row the vector of
%       the limb incontact "graspoing point" with the robot base
% OUTPUT: 
% - grasp_matrix = [6 x (N_limb_contact*N_joint)] matrix, representing grasp
%                  relation
function grasp_matrix = compute_grasp_matrix(r)
    grasp_matrix = [];
    r_new = [];
    for i=1:length(r)
        if r(i,1) == 0 & r(i,2) == 0  & r(i,3) == 0     
            % don't save it
        else
            r_new = [r_new; r(i,:)];
        end
    end
    for i=1:length(r_new)
        R(:,:,i) = [0 -r_new(i,3) r_new(i,2); r_new(i,3) 0 -r_new(i,1); -r_new(i,2) r_new(i,1) 0];
        W(:,:,i) = [eye(3), zeros(3,3); R(:,:,i), eye(3)];
        grasp_matrix = [grasp_matrix, W(:,:,i)];
    end
end