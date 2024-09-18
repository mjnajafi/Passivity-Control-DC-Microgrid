% This function designs a centralized local control strategy for a microgrid
% comprising multiple Distributed Generators (DGs) and transmission lines.
% The function computes the passivity parameters (P, K, nu, rho, gamma) for
% each DG and line by solving a series of Linear Matrix Inequalities (LMIs).
% The goal is to ensure the stability and passivity of the entire system.

function [DG,Line,statusLocalController] = centralizedLocalControlDesign(DG,Line,B_il,BarGamma,piVals,plVals)

numOfDGs = size(B_il,1);
numOfLines = size(B_il,2);

epsilon = 0.000001; % Minimum value
useNewCon8 = 0;
debugMode = 0;

% Create LMI variables necessary for (66)
%% Variables corresponding to DGs like P_i, K_i, nu_i, rhoTilde_i, gammaTilde_i
for i = 1:1:numOfDGs
    P_i{i} = sdpvar(3, 3, 'symmetric');
    K_i{i} = sdpvar(1, 3, 'full');
    nu_i{i} = sdpvar(1, 1, 'full');
    rho_i{i} = sdpvar(1, 1, 'full'); % Representing: rho (for newCon8)
    rhoTilde_i{i} = sdpvar(1, 1, 'full'); % Representing: 1/rho
    gammaTilde_i{i} = sdpvar(1, 1,'full');
end

% Variables corresponding to Lines like P_l, nu_l, rho_l
for l = 1:1:numOfLines
    P_l{l} = sdpvar(1, 1, 'symmetric');
    nu_l{l} = sdpvar(1, 1, 'full');
    rho_l{l} = sdpvar(1, 1, 'full'); 
end

constraints = [];
constraintTags = {}; % Cell array to hold tags
constraintMats = {}; % Cell array to hold matrices

%% Combine constraints over all DGs (66a-Part1, 66b,66d,66e)

for i = 1:1:numOfDGs

    Ai = DG{i}.A;
    Bi = DG{i}.B;
    Ii = eye(size(Ai));

    
    tagName = ['gammaTilde_',num2str(i),'_low'];
    constraintTags{end+1} = tagName;
    con0_1 = tag(gammaTilde_i{i} >= epsilon, tagName);
    constraintMats{end+1} = gammaTilde_i{i};

    tagName = ['gammaTilde_',num2str(i),'_high'];
    constraintTags{end+1} = tagName;
    con0_2 = tag(gammaTilde_i{i} <= BarGamma, tagName);
    constraintMats{end+1} = gammaTilde_i{i};

    % Constraint (66a-Part1)
    tagName = ['P_',num2str(i)];
    constraintTags{end+1} = tagName;
    con1 = tag(P_i{i} >= epsilon*Ii, tagName);
    constraintMats{end+1} = P_i{i};
    
    % Constraint (66b)
    DMat = rhoTilde_i{i} * eye(3);
    MMat = [P_i{i}, zeros(3)];
    ThetaMat = [-Ai*P_i{i} - P_i{i}'*Ai' - Bi*K_i{i} - K_i{i}'*Bi', -eye(3) + 0.5*P_i{i};
                -eye(3) + 0.5*P_i{i}, -nu_i{i}*eye(3)];
    W = [DMat, MMat; MMat', ThetaMat];
    
    tagName = ['W_',num2str(i)];
    constraintTags{end+1} = tagName;
    con2 = tag(W >= epsilon*eye(size(W)), tagName);
    constraintMats{end+1} = W;
    
    % Constraint (66d)
    p_i{i} = piVals(i); % predefined value

    tagName = ['nu_',num2str(i),'_low'];
    constraintTags{end+1} = tagName;
    con3_1 = tag(nu_i{i} >= -gammaTilde_i{i}/p_i{i}, tagName);
    constraintMats{end+1} = -gammaTilde_i{i}/p_i{i};

    tagName = ['nu_',num2str(i),'_high'];
    constraintTags{end+1} = tagName;
    con3_2 = tag(nu_i{i} <= -epsilon, tagName);
    constraintMats{end+1} = nu_i{i};
    

    % Constraint (66e)
    tagName = ['rhoTilde_',num2str(i),'_low'];
    constraintTags{end+1} = tagName;
    con4_1 = tag(rhoTilde_i{i} >= epsilon, tagName);
    constraintMats{end+1} = rhoTilde_i{i};

    tagName = ['rhoTilde_',num2str(i),'_high1'];
    constraintTags{end+1} = tagName;
    con4_21 = tag(rhoTilde_i{i} <= p_i{i}, tagName);
    constraintMats{end+1} = p_i{i};
    
    tagName = ['rhoTilde_',num2str(i),'_high2'];
    constraintTags{end+1} = tagName;
    con4_22 = tag(rhoTilde_i{i} <= 4*gammaTilde_i{i}/p_i{i}, tagName);
    constraintMats{end+1} = 4*gammaTilde_i{i}/p_i{i};

    % New con8:
    if useNewCon8
        tagName = ['rho_',num2str(i),'_low'];
        constraintTags{end+1} = tagName;
        con4_31 = tag(rho_i{i} >= 1/max(p_i{i}, 4*BarGamma/p_i{i}), tagName);
        constraintMats{end+1} = 1/max(p_i{i}, 4*BarGamma/p_i{i});
    
        tagName = ['rho_',num2str(i),'_high'];
        constraintTags{end+1} = tagName;
        con4_32 = tag(rho_i{i} <= 1/epsilon, tagName);
        constraintMats{end+1} = rho_i{i};
    
        tagName = ['rho_',num2str(i),'_receprocity'];
        constraintTags{end+1} = tagName;
        con4_33 = tag([rho_i{i}, 1; 1, rhoTilde_i{i}] >= 0, tagName); % rho_i rhoTilde_i >= 1
        constraintMats{end+1} = [rho_i{i}, 1; 1, rhoTilde_i{i}];
    end

    % Collecting Constraints
    if useNewCon8
        constraints = [constraints, con0_1, con0_2, con1, con2, con3_1, con3_2, con4_1, con4_21, con4_22, con4_31, con4_32, con4_33];
    else
        constraints = [constraints, con0_1, con0_2, con1, con2, con3_1, con3_2, con4_1, con4_21, con4_22];
    end
 end


%% Combine constraints over all Lines (66a-Part2, 66c)
for l = 1:1:numOfLines

    Rl = Line{l}.R;
    Ll = Line{l}.L;
    Il = eye(1);
    % Constraint (66a-Part2)

    tagName = ['PBar_',num2str(l)];
    constraintTags{end+1} = tagName;
    con5 = tag(P_l{l} >= epsilon*Il, tagName);
    constraintMats{end+1} = P_l{l};
    
    p_l{l} = plVals(l); %1/numOfLines;  % predefined value

    
    % Constraint (66c)
    W = [(2*P_l{l}*Rl)/Ll - rho_l{l}, -P_l{l}/Ll + 1/2;
         -P_l{l}/Ll + 1/2, -nu_l{l}];

    tagName = ['WBar_',num2str(l)];
    constraintTags{end+1} = tagName;
    con6 = tag(W >= epsilon*eye(size(W)), tagName);
    constraintMats{end+1} = W;
    
    tagName = ['nuBar_',num2str(l),'_high'];
    constraintTags{end+1} = tagName;
    con7_0 = tag(nu_l{l} <= -epsilon, tagName);
    constraintMats{end+1} = nu_l{l};

    tagName = ['rhoBar_',num2str(l),'low'];
    constraintTags{end+1} = tagName;
    con7_1 = tag(rho_l{l} >= epsilon, tagName);
    constraintMats{end+1} = rho_l{l};

    % Collecting Constraints
    constraints = [constraints, con5, con6, con7_0, con7_1];
end


%% Combine all mixed constraints (66f, 66g)
%%%% Comment: Debug these constraints, why their addition affects the
%%%% resulting passivity properties...?
for i = 1:1:numOfDGs
    for l = 1:1:numOfLines

        Ct = DG{i}.C;

        if B_il(i,l) ~= 0
       
           % Constraint (66f)
           tagName = ['rhoBar_',num2str(l),'_low1_',num2str(i)];
           constraintTags{end+1} = tagName;
           con7_2 = tag(rho_l{l} >= -(p_i{i}*nu_i{i})/(p_l{l}*Ct^2), tagName);
           constraintMats{end+1} = -(p_i{i}*nu_i{i})/(p_l{l}*Ct^2);
           
           tagName = ['rhoBar_',num2str(l),'_low2_',num2str(i)];
           constraintTags{end+1} = tagName;
           con7_3 = tag(rho_l{l} >= ((rhoTilde_i{i})/(p_i{i}*p_l{l}))*((p_i{i}/(2*Ct))-(p_l{l}/2))^2, tagName);
           constraintMats{end+1} = ((rhoTilde_i{i})/(p_i{i}*p_l{l}))*((p_i{i}/(2*Ct))-(p_l{l}/2))^2;
                
           constraints = [constraints, con7_2, con7_3];

           if useNewCon8
               % New con8 (66g):
               tagName = ['nuBar_',num2str(l),'_low_',num2str(i)];
               constraintTags{end+1} = tagName;
               con8New = nu_l{l} >= -p_i{i}*rho_i{i}/p_l{l};
               constraintMats{end+1} = -p_i{i}*rho_i{i}/p_l{l};
           else
               % Constraint (66g)lue
               n = 1;          % Number of intervals
               rho_max = min(p_i{i}, 4*BarGamma/p_i{i});
               rho_min = rho_max/1000;
    
               delta_i = (rho_max - rho_min) / n;
               
               % con8Test{i,l} = nu_l{l} +  p_i{i}/(rhoTilde_i{i}*p_l{l}); % This needs to be positive for the global controller to be feasible
    
               % Initialize cell array to store individual constraints
               con8 = [];
    
               % Loop over each k from 1 to n to create individual constraints
               tilde_rho_i_prev = rho_min;
               tilde_y_i_prev = -p_i{i} / (p_l{l} * tilde_rho_i_prev);
                
               for k = 1:n
    
                   % Compute tilde_rho_i^k
                   tilde_rho_i_k = rho_min + k * delta_i;
    
                   % Compute tilde_y_i^k
                   tilde_y_i_k = -p_i{i} / (p_l{l} * tilde_rho_i_k);
    
                   % Compute m_k and c_k
                   m_k = (tilde_y_i_k - tilde_y_i_prev) / delta_i;
                   c_k = tilde_y_i_k - m_k * tilde_rho_i_k;
    
                   % Define Constraint (66g)
                   tagName = ['nuBar_',num2str(l),'_low',num2str(n),'_',num2str(i)];
                   constraintTags{end+1} = tagName;
                   con8_k = tag(nu_l{l} >= m_k * rhoTilde_i{i} + c_k, tagName);
                   constraintMats{end+1} = m_k * rhoTilde_i{i} + c_k;
    
                   con8 = [con8, con8_k];
                   
                   % Compute tilde_rho_i^{k-1} and tilde_y_i^{k-1}
                   tilde_rho_i_prev = tilde_rho_i_k;
                   tilde_y_i_prev = tilde_y_i_k;
               end
           end

           if useNewCon8
               constraints = [constraints, con8New];
           else
               constraints = [constraints, con8];
           end
           
        end
    end
end



%% Solve the LMI problem (66)

costGamma = 0;
for  i = 1:numOfDGs
    if useNewCon8
        costGamma = costGamma + (-1000*nu_i{i}+rhoTilde_i{i}) + gammaTilde_i{i} + trace(P_i{i}) - 1000000*rho_i{i};
    else
        costGamma = costGamma + (-1000*nu_i{i}+rhoTilde_i{i}) + gammaTilde_i{i} + trace(P_i{i});
    end
end
for l = 1:numOfLines
    costGamma = costGamma + (-1000*nu_l{l}-rho_l{l}) + 1*trace(P_l{l});
end

% Defining costfunction
costFunction = 1*costGamma; % Play with this choice

solverOptions = sdpsettings('solver', 'mosek', 'verbose', 0, 'debug', 0);

sol = optimize(constraints, costFunction, solverOptions);

statusLocalController = sol.problem == 0;

% for i = 1:numOfDGs
%     % Get the numerical values for P_i and K_i
%     P_iVal = value(P_i{i});  
%     K_iVal = value(K_i{i});
% 
%     % Calculate K0 for each DG
%     if P_iVal ~= 0  % Check to avoid division by zero
%         DG{i}.K0 = K_iVal / P_iVal;  % K0 is defined as K_iVal divided by P_iVal
%     else
%         DG{i}.K0 = K_iVal;  % Keep original value if P_iVal is zero
%     end
% 
%     % Calculate min and max of K0 before thresholding
%     min_K0 = min(DG{i}.K0);
%     max_K0 = max(DG{i}.K0);
% 
%     % Display the min and max values
%     fprintf('DG %d: Min K0 = %.4f, Max K0 = %.4f\n', i, min_K0, max_K0);
% 
%     % Set threshold
%     threshold = 0.999 * max_K0;  % Calculate the threshold
%     fprintf('Threshold for DG %d: %.4f\n', i, threshold);  % Display threshold
% 
%     % Check conditions and assign zero if needed for each element
%     for k = 1:length(DG{i}.K0)  % Iterate over the elements of K0
%         if abs(DG{i}.K0(k)) < threshold  % Check condition based on threshold
%             fprintf('Assigning DG %d, K0[%d] from %.4f to 0\n', i, k, DG{i}.K0(k));  % Debug output
%             DG{i}.K0(k) = 0;  % Assign to zero
%         end
%     end
% 
%     % Display the updated K0 values for verification
%     fprintf('Updated K0 for DG %d: ', i);
%     disp(DG{i}.K0);
% end







%% Extract variable values
for i = 1:1:numOfDGs
    P_iVal = value(P_i{i});
    K_iVal = value(K_i{i});
    nu_iVal = value(nu_i{i});
    rhoTilde_iVal = value(rhoTilde_i{i});
    rho_iVal = 1 / rhoTilde_iVal;
    gammaTilde_iVal = value(gammaTilde_i{i});

    % update DG
    DG{i}.P0 = P_iVal;
    DG{i}.K0 = K_iVal/P_iVal;
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

%% Display violated constraints by tag
% Check feasibility
if debugMode
    feasibility = check(constraints);
    
    % Combine tags and feasibility into one array for sorting
    combinedList = [feasibility(:), (1:length(feasibility))'];
    
    % Sort based on the first column (feasibility values)
    sortedList = sortrows(combinedList, 1);  % Sort by feasibility, ascending order
    
    % Printing
    for i = 1:length(sortedList)
        idx = sortedList(i, 2);  % Get the original index of the constraint
        if feasibility(idx) < -1e-6
            disp(['Constraint "', constraintTags{idx}, '" is violated by ', num2str(feasibility(idx)), ' .']);
            W_val = value(constraintMats{idx})
            for j = 1:size(W_val, 1)
                submatrix = W_val(1:j, 1:j);  % Extract principal minor
                if det(submatrix) < 0
                    disp(['Principal minor ', num2str(j), ' is not positive semi-definite.']);
                end
            end
        else
            disp(['Constraint "', constraintTags{idx}, '" is satisfied by ',num2str(feasibility(idx)),' .']);
            W_val = value(constraintMats{idx})
        end
    end

    % Checking the nonlinear constraint
    for i = 1:1:numOfDGs
        for l = 1:1:numOfLines

            Ct = DG{i}.C;

            if B_il(i,l) ~= 0
                % del1_il = value(rho_l{l} +  p_i{i}*nu_i{i}/(Ct^2*p_l{l}))
                nonLinCons = value(nu_l{l} +  p_i{i}/(rhoTilde_i{i}*p_l{l}))
                del3 = value ( (rho_l{l}*Ct^2/(-nu_i{i})) - ((-nu_l{l})*rhoTilde_i{i}))
                
                T0 = Ct*(Ct*rho_l{l}/rhoTilde_i{i} + 2);
                T1 = (-nu_l{l})*rhoTilde_i{i};
                T2 = (-nu_i{i})/rho_l{l};
                del4 = value(T0-(T1+T2))

            end
        end
    end

end

end

