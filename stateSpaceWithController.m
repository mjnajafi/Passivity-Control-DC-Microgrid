function dx = stateSpaceWithController(x, u)
    
    V_i = x(1);
    I_ti = x(2);
    v_i = x(3);
    
    u = K0{1}*x(1)+K0{2}*x(2)+K0{3}*x(3);

    % Calculate the derivative of each state
    dV_i = -Y_Li/C_ti * V_i + 1/C_ti * I_ti;
    dI_ti = -1/L_ti * V_i - R_ti/L_ti * I_ti + 1/L_ti * u;
    dv_i = V_i;


    
   
    dx = [dV_i; dI_ti; dv_i];
end
