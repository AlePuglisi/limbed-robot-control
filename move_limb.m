% Use this function to recompute a single robot limb motion, when not in
% contact

% INPUT: 
% ROBOT = robot described as list of SerialLink 
% - q_in = currenty robot configuration 
% - i_limb = index of the rised limb to move
% - x = x motion of the limb in world coordinates
% - y = y motion of the limb in world coordinates
% - z = z motion of the limb in world coordinates
function q_new = move_limb(ROBOT, q_in, i_limb, x, y, z)
    names = ["LF"; "LH"; "RH"; "RF"];
    disp("Moving Limb " + names(i_limb,:) + " by x = " + num2str(x) + " | y = " + num2str(y) + " | z = " + num2str(z) + " (in World Frame)");
    t_limb = (ROBOT(i_limb).fkine(q_in(i_limb, :)).R)' * (ROBOT(i_limb).base.R)' *  [x y z]'
    T_ee_tool = transl(t_limb(1), t_limb(2), t_limb(3));
    T_limb = (ROBOT(i_limb).fkine(q_in(i_limb,:)).T)*T_ee_tool
    Mask = [ 1 1 1 0 0 0 ];
    q_new = q_in
    q_new(i_limb, :) = ROBOT(i_limb).ikine(T_limb, 'q0', q_in(i_limb, :), 'mask', Mask);
end