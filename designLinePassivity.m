function [Line, statusLine] = designLinePassivity(Line, DG, B_il, BarGamma, piVals,plVals,rhoTilde_i, nu_i)

numOfLines = size(B_il, 2);
numOfDGs = length(DG);

% Create LMI variables for lines
for l = 1:numOfLines
    P_l{l} = sdpvar(1, 1, 'symmetric');
    nu_l{l} = sdpvar(1, 1, 'full');
    rho_l{l} = sdpvar(1, 1, 'full'); 
end

constraints = [];

% Combine constraints over all Lines
for l = 1:numOfLines
    Rl = Line{l}.R;
    Ll = Line{l}.L;
    
    % Constraint (66a-Part2)
    con5 = P_l{l} >= 0;
    
    p_l{l} = plVals(l); % predefined value or derived from other sources

    % Constraint (66c)
    Z = [(2*P_l{l}*Rl)/Ll - rho_l{l}, -P_l{l}/Ll + 1/2;
         -P_l{l}/Ll + 1/2, -nu_l{l}];

    con6 = Z >= 0;
     
    % Collecting Constraints
    constraints = [constraints, con5, con6];
end

% Combine all mixed constraints
for i = 1:numOfDGs
    for l = 1:numOfLines
        Ct = DG{i}.C;

        if B_il(i, l) ~= 0
           
           % Constraint (66f)
           con7_1 = rho_l{l} >= 0; 
           con7_2 = rho_l{l} >= -(piVals(i)*nu_i{i})/(p_l{l}*Ct^2);
           con7_3 = rho_l{l} >= ((rhoTilde_i{i})/(piVals(i)*p_l{l}))*((piVals(i)/(2*Ct))-(p_l{l}/2))^2;
               
           % Constraint (66g)
           epsilon = 0.001; % Minimum value
           n = 10;          % Number of intervals
           rho_min = epsilon;
           rho_max = min(piVals(i), 4*BarGamma/piVals(i));
           delta_i = (rho_max - rho_min) / n;
           
           % Initialize cell array to store individual constraints
           con8 = [];

           % Loop over each k from 1 to n to create individual constraints
           tilde_rho_i_prev = rho_min;
           tilde_y_i_prev = -piVals(i) / (p_l{l} * tilde_rho_i_prev);

           for k = 1:n
               % Compute tilde_rho_i^k
               tilde_rho_i_k = rho_min + (k - 1) * delta_i;
    
               % Compute tilde_y_i^k
               tilde_y_i_k = -piVals(i) / (p_l{l} * tilde_rho_i_k);
    
               % Compute m_k and c_k
               m_k = (tilde_y_i_k - tilde_y_i_prev) / delta_i;
               c_k = tilde_y_i_k - m_k * tilde_rho_i_k;
    
               % Define Constraint (66g)
               con8_k = nu_l{l} >= m_k * rhoTilde_i{i} + c_k;
               con8 = [con8, con8_k];

               % Compute tilde_rho_i^{k-1} and tilde_y_i^{k-1}
               tilde_rho_i_prev = tilde_rho_i_k;
               tilde_y_i_prev = tilde_y_i_k;
           end

           % Collecting Constraints
           constraints = [constraints, con7_1, con7_2, con7_3, con8];
        end
    end
end

% Solve the LMI problem
costGamma = 0;
for i = 1:numOfLines
    costGamma = costGamma + (-nu_l{l}+rho_l{l});
end

% Defining cost function
costFunction = 1*costGamma;

solverOptions = sdpsettings('solver', 'mosek', 'verbose', 0);

sol = optimize(constraints, costFunction, solverOptions);

statusLine = sol.problem == 0;

% Extract variable values for Lines
for l = 1:numOfLines
    P_lVal = value(P_l{l});
    nu_lVal = value(nu_l{l});
    rho_lVal = value(rho_l{l});

    % Update Line
    Line{l}.P0 = P_lVal;
    Line{l}.nu = nu_lVal;
    Line{l}.rho = rho_lVal;
end

end
