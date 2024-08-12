function dx = stateSpaceWithController(x, u)
    % Unpack states
    V_i = x(1);
    I_ti = x(2);
    v_i = x(3);
    
    % Parameters defined from the previous context
    C_ti = 0.001; % Example value, replace with actual
    L_ti = 0.01;  % Example value, replace with actual
    R_ti = 0.05;  % Example value, replace with actual
    Y_Li = 0.02;  % Example value, replace with actual
    % P_Li = 0.03;  % Example value, replace with actual
    u_i0 = 1;     % Example value, replace with actual
    % Bar_I_Li = 0.1; % Example value, replace with actual
    % V_ri = 0.5;   % Example value, replace with actual
    
    % % % Matrix parameters, assuming B_il and u_ij are provided in `u`
    % B_il = u.B_il; % Matrix for sum of B_il*I_l, size should match
    % u_ij = u.u_ij; % Vector for sum of u_ij, size should match
    
    % % Calculate contributions
    % sum_B_il_I_l = sum(B_il .* u.I_l); % Sum over edges
    % sum_u_ij = sum(u_ij); % Sum over positive flow

    % Calculate the derivative of each state
    dV_i = -Y_Li/C_ti * V_i + 1/C_ti * I_ti;
    dI_ti = -1/L_ti * V_i - R_ti/L_ti * I_ti + 1/L_ti * u_i0;
    dv_i = V_i;

    % % Include additional contributions
    % dV_i = dV_i - Bar_I_Li / C_ti - sum_B_il_I_l / C_ti;
    % dI_ti = dI_ti + sum_u_ij / L_ti;
    % dv_i = dv_i - V_ri;

    % Pack derivatives into output vector
    dx = [dV_i; dI_ti; dv_i];
end
