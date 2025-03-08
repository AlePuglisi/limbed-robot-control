
function tau = approx_GravityLoad(limb, q, joint_num, contact)

r_cm_tilde = [];
m = [];

if contact == 1

    m_torso = 3.83/4;
    m_torso = 1;
    W = 0.225; 
    L = 0.225; 
    joint_num_contact = limb.n - joint_num;
    
    r_cm_tilde_j(1, :) = [W/2, 0 , 0, 1];
     m(1) = m_torso; 

     for i=1:joint_num
        r_cm_tilde_i(i+1,:) = [limb.links(limb.n-i+1).r, 1];
        m(i+1) = limb.links(limb.n-i+1).m;
    end


    T_j_i= [];
    r_cm_tilde_j = [];

    T_j_N = limb.A(1:joint_num_contact, q)^-1*limb.base*limb.fkine(q);
    T_j_torso =T_j_N.T*transl(W/2, 0, 0);
    r_cm_tilde_j(1,:) = [T_j_torso(1:3,4)', 1];

    for i=1:joint_num 
        T_j_i(:,:,i) = limb.A(1:joint_num_contact, q)^-1*limb.A(1:i, q);
        r_cm_tilde_j(i+1,:) = T_j_i(:,:,i)*r_cm_tilde_i(i+1,:)';
    end

    r_cm_j = r_cm_tilde_j(:,1:3)

    CoM = [];
    CoM(1) = r_cm_j(:,1)'*m'/sum(m);
    CoM(2) = r_cm_j(:,2)'*m'/sum(m);
    CoM(3) = r_cm_j(:,3)'*m'/sum(m);

    Mg_0 = [0, 0, sum(m)*9.81];
    Mg = (limb.A(1:joint_num_contact,q).R)'*Mg_0';

    tau = cross(Mg, CoM)
end

if contact == 0

     for i=joint_num:limb.n
        r_cm_tilde_i(i-joint_num+1,:) = [limb.links(i).r, 1];
        m(i-joint_num+1) = limb.links(i).m;
    end


    T_j_i= [];
    r_cm_tilde_j = [];

    for i=joint_num:limb.n
        T_j_i(:,:,i) = limb.A(1:joint_num, q)^-1*limb.A(1:i, q);
        r_cm_tilde_j(i-joint_num+1,:) = T_j_i(:,:,i)*r_cm_tilde_i(i-joint_num+1,:)';
    end

    r_cm_j = r_cm_tilde_j(:,1:3);

    CoM = [];
    CoM(1) = r_cm_j(:,1)'*m'/sum(m);
    CoM(2) = r_cm_j(:,2)'*m'/sum(m);
    CoM(3) = r_cm_j(:,3)'*m'/sum(m);

    Mg_0 = [0, 0, -sum(m)*9.81];
    Mg = (limb.A(1:joint_num,q).R)'*Mg_0';

    tau = cross(Mg, CoM);
end

end