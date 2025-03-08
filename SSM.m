function StaticStabilityMargin = SSM(ROBOT, q, contact_mask, T_base, normalized)
    N_limb = length(ROBOT);
     t_support = [];
    for i=1:N_limb
        if(contact_mask(i) == 1)
            t_support = [t_support; ROBOT(i).fkine(q(i,:)).t'];
        end      
    end   
    N_contact = sum(contact_mask);
       
    CoM = [];
    CoM(1) = T_base(1,4);
    CoM(2) = T_base(2,4);
    CoM(3) = sum(t_support(:,3))/N_contact;
    t_support = [t_support; t_support(1,:)];

    SMs = [];
    for i=1:N_contact
            SMs = [SMs, dist(CoM, [t_support(i,:); t_support(i+1,:)])];
    end
    StaticStabilityMargin = min(SMs);

    if normalized == 1
        t_support = [];
        q0 = zeros(N_limb,7);
        for i=1:N_limb
            q0(i,3) = pi/6 + pi/2; 
            q0(i,2) = -pi/6; 
             t_support = [t_support; ROBOT(i).fkine(q0(i,:)).t'];
        end
         t_support = [t_support; t_support(1,:)];
        SMs = [];
        for i=1:N_limb
            SMs = [SMs, dist(CoM, [t_support(i,:); t_support(i+1,:)])];
        end
        SSM_max = min(SMs);
        StaticStabilityMargin = (StaticStabilityMargin/SSM_max)*100;
    end
    
end

function d = dist(P, Perimeter_Edge)
    x_A = Perimeter_Edge(1,1);
    x_B = Perimeter_Edge(2,1);
    y_A = Perimeter_Edge(1,2);
    y_B = Perimeter_Edge(2,2);

    d = abs((x_B - x_A)*(y_A-P(2)) - ((x_A - P(1))*(y_B - y_A)))/sqrt((x_B-x_A)^2 + (y_B - y_A)^2);
end

