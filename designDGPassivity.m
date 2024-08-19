% This function designs the passivity parameters for multiple Distributed Generators (DGs)
% within a microgrid framework. It formulates and solves a series of Linear Matrix Inequalities
% (LMIs) to ensure that each DG meets the required passivity conditions.

function [DG, rhoTilde_i, nu_i, gammaTilde_i,statusDG] = designDGPassivity(DG, B_il, piVals)

numOfDGs = size(B_il, 1);

% Create LMI variables necessary for DG passivity
for i = 1:numOfDGs
    P_i{i} = sdpvar(3, 3, 'symmetric');
    K_i{i} = sdpvar(1, 3, 'full');
    nu_i{i} = sdpvar(1, 1, 'full');
    rhoTilde_i{i} = sdpvar(1, 1, 'full'); % Representing: 1/rho
    gammaTilde_i{i} = sdpvar(1, 1,'full');
end

constraints = [];

% Combine constraints over all DGs
for i = 1:numOfDGs
    Ai = DG{i}.A;
    Bi = DG{i}.B;

    % Constraint (66a-Part1)
    con1 = P_i{i} >= 0;
    
    % Constraint (66b)
    DMat = rhoTilde_i{i} * eye(3);
    MMat = [P_i{i}, zeros(3)];
    ThetaMat = [-Ai*P_i{i} - P_i{i}'*Ai' - Bi*K_i{i} - K_i{i}'*Bi', -eye(3) + 0.5*P_i{i};
                -eye(3) + 0.5*P_i{i}, -nu_i{i}*eye(3)];
    W = [DMat, MMat; MMat', ThetaMat];
    
    con2 = W >= 0;
    
    % Constraint (66d)
    p_i{i} = piVals(i); % Assume a predefined value or an input parameter
    con3_1 = -gammaTilde_i{i}/p_i{i} <= nu_i{i};
    con3_2 = nu_i{i} <= 0;
    
    % Constraint (66e)
    con4_1 = 0 <= rhoTilde_i{i};
    con4_21 = rhoTilde_i{i} <= p_i{i};
    con4_22 = rhoTilde_i{i} <= 4*gammaTilde_i{i}/p_i{i};
    
    % Collecting Constraints
    constraints = [constraints, con1, con2, con3_1, con3_2, con4_1, con4_21, con4_22];
end

% Solve the LMI problem
costGamma = 0;
for i = 1:numOfDGs
    costGamma = costGamma + (-nu_i{i}+rhoTilde_i{i}) + gammaTilde_i{i};
end

% Defining cost function
costFunction = 1*costGamma;

solverOptions = sdpsettings('solver', 'mosek', 'verbose', 0);

sol = optimize(constraints, costFunction, solverOptions);

statusDG = sol.problem == 0;

% Extract variable values

for i = 1:numOfDGs
    P_iVal = value(P_i{i});
    K_iVal = value(K_i{i});
    nu_iVal = value(nu_i{i});
    rhoTilde_iVal = value(rhoTilde_i{i});
    rho_iVal = 1 / rhoTilde_iVal;
    gammaTilde_iVal = value(gammaTilde_i{i});

    % Update DG
    DG{i}.P0 = P_iVal;
    DG{i}.K0 = K_iVal/P_iVal;
    DG{i}.nu = nu_iVal;
    DG{i}.rho = rho_iVal;
    DG{i}.gammaTilde0 = gammaTilde_iVal;
    
    
end

end
