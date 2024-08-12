% Filename: lineDynamics.m
function dx = lineDynamics(x_l, u_l)
    % Define parameters (constants)
    R_l = 0.05; % Example value for resistance
    L_l = 0.01; % Example value for inductance
    
    % State vector
    % x_l is the state vector: [I_l]
    
    % Input vector
    % u_l is the voltage difference: sum of V_i across the line
    
    % System matrices
    A_l = -R_l / L_l;  % Single-element matrix
    B_l = 1 / L_l;     % Single-element matrix
    
    % Calculate state derivatives
    dx = A_l * x_l + B_l * u_l;
end
