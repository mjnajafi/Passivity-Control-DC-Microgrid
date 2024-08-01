function [DG,Line] = centralizedLocalControlDesign(DG,Line,B_il)

%% Create LMI variables necessary for (66)
% Variables corresponding to DGs like P_i, K_i, nu_i, rhoTilde_i, gammaTilde_i
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
    p_i = 1;    % predefined value
    
    con3_1 = -gammaTilde_i{i}/p_i <= nu_i{i};
    con3_2 = nu_i{i} <= 0;
    

    % Constraint (66e)
    con4_1 = 0 <= rhoTilde_i{i};

    con4_21 = rhoTilde_i{i} <= p_i;
    con4_22 = rhoTilde_i{i} <= 4*gammaTilde_i{i}/p_i;
    
    % Collecting Constraints
    constraints = [constraints, con1];
    constraints = [constraints, con2];
    constraints = [constraints, con3_1];
    constraints = [constraints, con3_2];
    constraints = [constraints, con4_1];
    constraints = [constraints, con4_21];
    constraints = [constraints, con4_22];
end


%% Combine constraints over all Lines (66a-Part2, 66c)
for l = 1:1:numOfLines
    
    constraints = [constraints, newConstraints]
end


%% Combine all mixed constraints (66f, 66g)
for i = 1:1:numOfDGs
    for l = 1:1:numbOfLines
        % Add a constraint only if l in neighborhood of i
        constraints = [constraints, newConstraints]
    end
end



%% Solve the LMI problem (66)
costFunction = 0;
sol = optimize(constraints, solverOptions);




%% Extractt variable values
for i = 1:1:numOfDGs
    P_iVal = value(P_i{i})
    nu_iVal = value(nu_i{i})

    % update DG
    DG{i}.nu = nu_iVal;
end

for l = 1:1:numOfLine
    P_lVal = value(P_l{l})
    nu_lVal = value(nu_l{l})

    % update DG
    Line{l}.nu = nu_lVal;
end



end

