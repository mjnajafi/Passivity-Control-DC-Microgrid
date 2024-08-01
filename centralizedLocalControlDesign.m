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

    constraints = [constraints, newConstraints]
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

