function [DG,Line,statusLocalController] = centralizedLocalControlDesign(DG,Line,B_il,BarGamma,piVals,plVals)

numOfDGs = size(B_il,1);
numOfLines = size(B_il,2);

epsilon = 0.000001; % Minimum value
debugMode = 0; % Enable for debugging

% Create LMI variables
for i = 1:1:numOfDGs
    P_i{i} = sdpvar(3, 3, 'symmetric');
    Kp_i{i} = sdpvar(1, 1, 'full');  % Proportional gain
    Ki_i{i} = sdpvar(1, 1, 'full');  % Integral gain
    nu_i{i} = sdpvar(1, 1, 'full');
    rhoTilde_i{i} = sdpvar(1, 1, 'full'); % Representing: 1/rho
    gammaTilde_i{i} = sdpvar(1, 1,'full');
end

for l = 1:1:numOfLines
    P_l{l} = sdpvar(1, 1, 'symmetric');
    nu_l{l} = sdpvar(1, 1, 'full');
    rho_l{l} = sdpvar(1, 1, 'full'); 
end

constraints = [];
constraintTags = {};
constraintMats = {};

%% Combine constraints over all DGs (66a-Part1, 66b)
for i = 1:1:numOfDGs
    K_i{i} = [Kp_i{i}, 0, Ki_i{i}];  % This enforces [Kp, 0, Ki] structure
    
    Ai = DG{i}.A;
    Bi = DG{i}.B;
    Ii = eye(size(Ai));

    % Constraints on the passivity indices
    % tagName = ['nu_i_',num2str(i),'_negative'];
    % constraintTags{end+1} = tagName;
    % con_nu_i = tag(nu_i{i} <= -epsilon, tagName);  % nu_i should be negative
    % constraintMats{end+1} = nu_i{i};
    % 
    % tagName = ['rhoTilde_i_',num2str(i),'_positive'];
    % constraintTags{end+1} = tagName;
    % con_rho_i = tag(rhoTilde_i{i} >= epsilon, tagName);  % rhoTilde_i should be positive
    % constraintMats{end+1} = rhoTilde_i{i};

    tagName = ['gammaTilde_',num2str(i),'_low'];
    constraintTags{end+1} = tagName;
    con0_1 = tag(gammaTilde_i{i} >= epsilon, tagName);
    constraintMats{end+1} = gammaTilde_i{i};

    tagName = ['gammaTilde_',num2str(i),'_high'];
    constraintTags{end+1} = tagName;
    con0_2 = tag(gammaTilde_i{i} <= BarGamma, tagName);
    constraintMats{end+1} = gammaTilde_i{i};

    % Constraint 
    tagName = ['P_',num2str(i)];
    constraintTags{end+1} = tagName;
    con1 = tag(P_i{i} >= epsilon, tagName);
    constraintMats{end+1} = P_i{i};
    
    % Construct matrix W_i
    DMat = rhoTilde_i{i} * eye(3);
    MMat = [P_i{i}, zeros(3)];
    ThetaMat = [-Ai*P_i{i} - P_i{i}'*Ai' - Bi*K_i{i} - K_i{i}'*Bi', -eye(3) + 0.5*P_i{i};
                -eye(3) + 0.5*P_i{i}, -nu_i{i}*eye(3)];
    W_i = [DMat, MMat; MMat', ThetaMat];
    

    % constraints = [constraints, Kp_i{i} >= 0];
    % Create a slack variable with the SAME DIMENSIONS as W_i
    % slack_W = sdpvar(size(W_i,1), size(W_i,2), 'symmetric');
    
    tagName = ['W_',num2str(i)];
    constraintTags{end+1} = tagName;
    con2 = tag(W_i >= epsilon, tagName);
    constraintMats{end+1} = W_i;

    % Add constraints on the slack variable
    % constraints = [constraints, slack_W >= 0, trace(slack_W) <= 1];
    % constraints = [constraints, slack_W >= 0]; 

    
    % Adding constraints
    % constraints = [constraints, con_nu_i, con_rho_i, con0_1, con0_2, con1, con2];
    constraints = [constraints, con0_1, con0_2, con1, con2];
    % constraints = [constraints, con1, con2];

end

%% Combine constraints over all Lines (66a-Part2, 66c)
for l = 1:1:numOfLines
    Rl = Line{l}.R;
    Ll = Line{l}.L;
    Il = eye(1);
    
    % Constraints on the passivity indices
    % tagName = ['nu_l_',num2str(l),'_nonpositive'];
    % constraintTags{end+1} = tagName;
    % con_nu_l = tag(nu_l{l} <= 0, tagName);  % nu_l should be non-positive (≤ 0)
    % constraintMats{end+1} = nu_l{l};
    % 
    % tagName = ['rho_l_',num2str(l),'_positive'];
    % constraintTags{end+1} = tagName;
    % con_rho_l = tag(rho_l{l} >= epsilon, tagName);  % rho_l should be positive
    % constraintMats{end+1} = rho_l{l};
  
    % Constraint (66a-Part2)
    tagName = ['PBar_',num2str(l)];
    constraintTags{end+1} = tagName;
    con5 = tag(P_l{l} >= epsilon, tagName);
    constraintMats{end+1} = P_l{l};
    
    p_l{l} = plVals(l); % predefined value
    
    % Constraint (66c)
    W_l = [(2*P_l{l}*Rl)/Ll - rho_l{l}, -P_l{l}/Ll + 1/2;
         -P_l{l}/Ll + 1/2, -nu_l{l}];
    
    % Create a slack variable with the SAME DIMENSIONS as W_l
    slack_W_l = sdpvar(size(W_l,1), size(W_l,2), 'symmetric');

    tagName = ['WBar_',num2str(l)];
    constraintTags{end+1} = tagName;
    con6 = tag(W_l + slack_W_l >= epsilon, tagName);
    constraintMats{end+1} = W_l;
    
    % Add constraints on the slack variable
    % constraints = [constraints, slack_W_l >= 0, trace(slack_W_l) <= 1];
    constraints = [constraints, slack_W_l >= 0];

    
    % Collecting Constraints
    % constraints = [constraints, con_nu_l, con_rho_l, con5, con6];
    constraints = [constraints, con5, con6];

end

%% Implement the lumped LMI necessary condition from equation 83
for i = 1:1:numOfDGs
    for l = 1:1:numOfLines
        if B_il(i,l) ~= 0
            p_i = piVals(i);
            p_l = plVals(l);
            Ct = DG{i}.C;
            
            % Define the non-zero values directly
            B_il_val = B_il(i,l);
            
            % Define auxiliary variable for bilinear term
            xi_il = sdpvar(1,1,'full');
            s1 = sdpvar(1,1,'full');
            s2 = sdpvar(1,1,'full');
            
            % Add LMI constraint for auxiliary variable
            auxiliary_LMI = [
                1, nu_l{l}, rhoTilde_i{i};
                nu_l{l}, s1, xi_il;
                rhoTilde_i{i}, xi_il, s2
            ];
            
            tagName = ['AuxLMI_',num2str(i),'_',num2str(l)];
            constraintTags{end+1} = tagName;
            con_aux = tag(auxiliary_LMI >= epsilon, tagName);
            constraintMats{end+1} = auxiliary_LMI;
            constraints = [constraints, con_aux];
            
            % Building the LMI step by step with subblocks
            % Block (1,1)
            Mat_11 = -p_i*nu_i{i};
            Mat_1 = Mat_11;
            
            % Block (2,2)
            Mat_22 = -p_l*nu_l{l};
            
            % Block (1,2)
            Mat_12 = 0;
            
            % Combine to 2x2 matrix
            Mat_2 = [Mat_1, Mat_12;
                    Mat_12', Mat_22];
            
            % Block (3,3)
            Mat_33 = 1;
            
            % Block (1,3) and (2,3)
            Mat_13 = [0; 0];
            
            % Combine to 3x3 matrix
            Mat_3 = [Mat_2, Mat_13;
                    Mat_13', Mat_33];
            
            % Block (4,4)
            Mat_44 = p_i*rhoTilde_i{i};
            
            % Block (1,4), (2,4), and (3,4)
            Mat_14 = [0;
                      -B_il_val*xi_il*p_l;
                      rhoTilde_i{i}];
            
            % Combine to 4x4 matrix
            Mat_4 = [Mat_3, Mat_14;
                    Mat_14', Mat_44];
            
                   
            % Block (5,5)
            Mat_55 = p_l*rho_l{l};
            
            % Block (1,5), (2,5), (3,5), and (4,5)
            Mat_15 = [(p_i*nu_i{i}*B_il_val)/Ct;
                      0;
                      1;
                      (1/2*p_i*B_il_val*rhoTilde_i{i})/Ct - 1/2*B_il_val*p_l*rhoTilde_i{i}];
            
            % Combine to 5x5 matrix
            Mat_5 = [Mat_4, Mat_15;
                    Mat_15', Mat_55];
            
                     
            % Block (6,6)
            Mat_66 = gammaTilde_i{i};
            
            % Block (1,6), (2,6), (3,6), (4,6), and (5,6)
            Mat_16 = [-p_i*nu_i{i};
                      -p_l*nu_l{l};
                      0;
                      -1/2*p_i*rhoTilde_i{i};
                      -1/2*p_l];
            
            % Combine to final 6x6 matrix
            Mat_6 = [Mat_5, Mat_16;
                    Mat_16', Mat_66];
            
            % Create a slack variable with SAME DIMENSIONS as Mat_6
            % slack_Mat6 = sdpvar(size(Mat_6,1), size(Mat_6,2), 'symmetric');
            
            % Final LMI condition
            W = Mat_6;
            
            tagName = ['LMI_83_',num2str(i),'_',num2str(l)];
            constraintTags{end+1} = tagName;
            con_LMI_83 = tag(W >= epsilon, tagName);
            constraintMats{end+1} = W;
            
            % Add constraints on the slack variable
            % constraints = [constraints, slack_Mat6 >= 0, trace(slack_Mat6) <= 1];
            
            % Add to constraints
            constraints = [constraints, con_LMI_83];
        end
    end
end

%% Solve the LMI problem (66)
costGamma = 0;
slackPenalty = 10;  % High penalty to minimize slack usage

for i = 1:numOfDGs
    costGamma = costGamma + 1*rhoTilde_i{i} - 1*nu_i{i} + 1000*gammaTilde_i{i};
end

for l = 1:numOfLines
    costGamma = costGamma + 1*rho_l{l} - 1*nu_l{l}  + slackPenalty * trace(slack_W_l);
end



% Add a regularization term to penalize large controller gains
% for i = 1:numOfDGs
%     costGamma = costGamma + 1 * norm(K_i{i}, 'fro');  % Frobenius norm of the controller gain
% end

% solverOptions = sdpsettings('solver', 'mosek', 'verbose', 1, 'debug', 0);

solverOptions = sdpsettings('solver', 'mosek', 'verbose', 2, 'debug', debugMode, ...
                           'mosek.MSK_DPAR_INTPNT_CO_TOL_REL_GAP', 1e-5, ...
                           'mosek.MSK_DPAR_INTPNT_CO_TOL_PFEAS', 1e-6, ...
                           'mosek.MSK_DPAR_INTPNT_CO_TOL_DFEAS', 1e-6);

sol = optimize(constraints, costGamma, solverOptions);


statusLocalController = sol.problem == 0;

% Check for diagnostic information if infeasible
if sol.problem ~= 0
    disp(['Problem status: ', yalmiperror(sol.problem)]);
    disp('Problem is infeasible or numerical issues occurred. Checking diagnostics...');
    
    % Try to identify problematic constraints
    if debugMode
        feasibility = check(constraints);
        
        % Combine tags and feasibility into one array for sorting
        combinedList = [feasibility(:), (1:length(feasibility))'];
        
        % Sort based on the first column (feasibility values)
        sortedList = sortrows(combinedList, 1);
        
        % Printing constraint status
        for i = 1:min(10, length(sortedList))  % Print top 10 most violated constraints
            idx = sortedList(i, 2);
            if feasibility(idx) < -1e-6
                disp(['Constraint "', constraintTags{idx}, '" is violated by ', num2str(feasibility(idx)), ' .']);
            end
        end
    end
end

%% Extract variable values if solution was found
if statusLocalController
    for i = 1:1:numOfDGs
        P_iVal = value(P_i{i});
        Kp_iVal = value(Kp_i{i});
        Ki_iVal = value(Ki_i{i});
        nu_iVal = value(nu_i{i});
        rhoTilde_iVal = value(rhoTilde_i{i});
        rho_iVal = 1 / rhoTilde_iVal;
        gammaTilde_iVal = value(gammaTilde_i{i});

        Kp0 = Kp_iVal / P_iVal(1,1);  % Use only the (1,1) element of P for Kp
        Ki0 = Ki_iVal / P_iVal(3,3);  % Use only the (3,3) element of P for Ki
            
        % Construct K0 with guaranteed zero middle element
        K0 = [Kp0, 0, Ki0];

        % update DG
        DG{i}.P0 = P_iVal;
        DG{i}.K0 = K0;
        DG{i}.nu = nu_iVal;
        DG{i}.rho = rho_iVal;
        DG{i}.gammaTilde0 = gammaTilde_iVal;
    end

    for l = 1:1:numOfLines
        P_lVal = value(P_l{l});
        nu_lVal = value(nu_l{l});
        rho_lVal = value(rho_l{l});

        % update Line
        Line{l}.P0 = P_lVal;
        Line{l}.nu = nu_lVal;
        Line{l}.rho = rho_lVal;
    end
end
end