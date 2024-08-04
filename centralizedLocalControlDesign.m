function [DG,Line] = centralizedLocalControlDesign(DG,Line,B_il,numOfDGs,numOfLines)

% Create LMI variables necessary for (66)
%% Variables corresponding to DGs like P_i, K_i, nu_i, rhoTilde_i, gammaTilde_i
for i = 1:1:numOfDGs
    P_i{i} = sdpvar(3, 3, 'symmetric');
    K_i{i} = sdpvar(1, 3, 'full');
    nu_i{i} = sdpvar(1, 1, 'full');
    rhoTilde_i{i} = sdpvar(1, 1, 'full'); % Representing: 1/rho
    gammaTilde_i{i} = sdpvar(1,1,'full');
end

% Variables corresponding to Lines like P_l, nu_l, rho_l
for l = 1:1:numOfLines
    P_l{l} = sdpvar(1, 1, 'symmetric');
    nu_l{l} = sdpvar(1, 1, 'full');
    rho_l{l} = sdpvar(1, 1, 'full'); 
end

constraints = [];
%% Combine constraints over all DGs (66a-Part1, 66b,66d,66e)

for i = 1:1:numOfDGs

    

    % Constraint (66a-Part1)
    con1 = P_i{i} >= 0;
    
    % Constraint (66b)
    DMat = rhoTilde_i{i} * eye(3);
    MMat = [P_i{i}, zeros(3)];
    ThetaMat = [-DG.A{i}*P_i{i} - P_i{i}'*DG.A{i}' - DG.B{i}*K_i{i} - K_i{i}'*DG.B{i}', -eye(3) + 0.5*P_i{i};
                -eye(3) + 0.5*P_i{i}, -nu_i{i}*eye(3)];
    W = [DMat, MMat; MMat', ThetaMat];
    
    con2 = W >= 0;
    
    % Constraint (66d)
    p_i{i} = 1/numOfDGs;    % predefined value
    
    con3_1 = -gammaTilde_i{i}/p_i{i} <= nu_i{i};
    con3_2 = nu_i{i} <= 0;
    

    % Constraint (66e)
    con4_1 = 0 <= rhoTilde_i{i};

    con4_21 = rhoTilde_i{i} <= p_i{i};
    con4_22 = rhoTilde_i{i} <= 4*gammaTilde_i{i}/p_i{i};
    
    % Collecting Constraints
    constraints = [constraints, [con1, con2, con3_1, con3_2, con4_1, con4_21, con4_22]];
    % constraints = [constraints, con1];
    % constraints = [constraints, con2];
    % constraints = [constraints, con3_1];
    % constraints = [constraints, con3_2];
    % constraints = [constraints, con4_1];
    % constraints = [constraints, con4_21];
    % constraints = [constraints, con4_22];
end


%% Combine constraints over all Lines (66a-Part2, 66c)
for l = 1:1:numOfLines
    

    % Constraint (66a-Part2)
    con5 = P_l{l} >= 0;
    
    p_l{l} = 1/numOfLines;

    % Constraint (66c)
    Z = [(2*P_l{l}*line.R{l})/line.L{l} - rho_l{l}, -P_l{l}/line.L{l} + 1/2;
         -P_l{l}/line.L{l} + 1/2, -nu_l{l}];

    con6 = Z >= 0;
     
    % Collecting Constraints
    constraints = [constraints, [con5, con6]];
    % constraints = [constraints, con5];
    % constraints = [constraints, con6]; 
end


%% Combine all mixed constraints (66f, 66g)
for i = 1:1:numOfDGs
    for l = 1:1:numbOfLines

        if B_il(i,l) ~= 0
       
           % Constraint (66f)
           con7_1 = rho_l{l} >= -(p_i{i}*nu_i{i})/(p_l{l}*DG.C{i}^2);
           con7_2 = rho_l{l} >= (rhoTilde_i{i})/(p_i{i}*p_l{l})*((p_i{i})/(2*DG.C{i})-((p_l{l})/2))^2;
    
                
           % Constraint (66g)
           epsilon = 0.001;
           n = 10;
           BarGamma = 5;
           rho_min = epsilon;
           rho_max = min(p_i{i}, 4*BarGamma/p_i{i});
           delta_i = (rho_max - rho_min) / n;
           
            
           % Initialize cell array to store individual constraints
           con8 = cell(1, n);

           % Loop over each k from 1 to n to create individual constraints
           for k = 1:n

               % Compute tilde_rho_i^k
               tilde_rho_i_k = rho_min + (k - 1) * delta_i;
    
               % Compute tilde_y_i^k
               tilde_y_i_k = -p_i{i} / (p_l{l} * tilde_rho_i_k);
    
               % Compute tilde_rho_i^{k-1} and tilde_y_i^{k-1}
               if k == 1
    
                   % For k = 1, use tilde_rho_i^0 = rho_min
                   tilde_rho_i_prev = rho_min;
                   tilde_y_i_prev = -p_i{i} / (p_l{l} * tilde_rho_i_prev);
               else
                   % For k > 1, compute tilde_rho_i^{k-1}
                   tilde_rho_i_prev = rho_min + (k - 2) * delta_i;
                   tilde_y_i_prev = -p_i{i} / (p_l{l} * tilde_rho_i_prev);
               end
    
               % Compute m_k and c_k
               m_k = (tilde_y_i_k - tilde_y_i_prev) / delta_i;
               c_k = tilde_y_i_k - m_k * tilde_rho_i_k;
    
               % Define Constraint (66g)
               con8{k} = nu_l{l} >= m_k * rhoTilde_i{i} + c_k;
           end

               % Define the final constraint
               min_constraint = min(cell2mat(con8)); % Combine all constraints using min

               % Collecting Constraints
               constraint = [constraint, [con7_1, con7_2, min_constraint]];                 
        end
    end
end



%% Solve the LMI problem (66)

% Defining costfunction
costFunction = 0;

solverOptions = sdpsettings('solver', 'mosek', 'verbose', 0);

sol = optimize(constraints, costFunction, solverOptions);

status = sol.problem == 0;


%% Extractt variable values
for i = 1:1:numOfDGs
    P_iVal = value(P_i{i});
    K_iVal = value(K_i{i});
    nu_iVal = value(nu_i{i});
    rhoTilde_iVal = value(rhoTilde_i{i});
    gammaTilde_iVal = value(gammaTilde_i{i});

    % update DG
    DG{i}.P = P_iVal;
    DG{i}.K = K_iVal;
    DG{i}.nu = nu_iVal;
    DG{i}.rhoTilde = rhoTilde_iVal;
    DG{i}.gammaTilde = gammaTilde_iVal;
end

for l = 1:1:numOfLine
    P_lVal = value(P_l{l});
    nu_lVal = value(nu_l{l});
    rho_lVal = value(rho_l{l});

    % update Line
    Line{l}.P = P_lVal;
    Line{l}.nu = nu_lVal;
    Line{l}.rho = rho_lVal;
end



end

