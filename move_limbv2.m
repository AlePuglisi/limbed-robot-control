% Use this function to recompute a single robot limb motion, when not in
% contact

% INPUT: 
% ROBOT = robot described as list of SerialLink 
% - q_in = currenty robot configuration 
% - i_limb = index of the rised limb to move
% - x = x motion of the limb in world coordinates
% - y = y motion of the limb in world coordinates
% - z = z motion of the limb in world coordinates
function q_new = move_limbv2(ROBOT, q_in, i_limb, x, y, z, transform)
    names = ["LF"; "LH"; "RH"; "RF"];
    if length(ROBOT) == 6
        names = ["LF"; "LM"; "LB"; "RF"; "RM"; "RB"];
    end
    disp("Moving Limb " + names(i_limb,:) + " by x = " + num2str(x) + " | y = " + num2str(y) + " | z = " + num2str(z) + " (in World Frame)");
    t_limb = (ROBOT(i_limb).fkine(q_in(i_limb, :)).R)' * (ROBOT(i_limb).base.R)' *  [x y z]';
    T_ee_tool = transl(t_limb(1), t_limb(2), t_limb(3));
    %  if transform == "W "
    %     T_ee_tool = T_ee_tool*troty(pi/2)*trotz(pi)
    %     %T_ee_tool = T_ee_tool*troty(pi/2)*trotz(pi);
    % end
    T_limb = (ROBOT(i_limb).fkine(q_in(i_limb,:)).T)*T_ee_tool;
    Mask = [ 1 1 1 1 1 1 ];
    q_new = q_in;
    q_new(i_limb, :) = ROBOT(i_limb).ikine(T_limb, 'q0', q_in(i_limb, :), 'mask', Mask);
end