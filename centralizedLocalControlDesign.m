% This function designs a centralized local control strategy for a microgrid
% comprising multiple Distributed Generators (DGs) and transmission lines.
% The function computes the passivity parameters (P, K, nu, rho, gamma) for
% each DG and line by solving a series of Linear Matrix Inequalities (LMIs).
% The goal is to ensure the stability and passivity of the entire system.

function [DG,Line,statusLocalController] = centralizedLocalControlDesign(DG,Line,B_il,BarGamma,piVals,plVals)

numOfDGs = size(B_il,1);
numOfLines = size(B_il,2);

% Create LMI variables necessary for (66)
%% Variables corresponding to DGs like P_i, K_i, nu_i, rhoTilde_i, gammaTilde_i
for i = 1:1:numOfDGs
    P_i{i} = sdpvar(3, 3, 'symmetric');
    K_i{i} = sdpvar(1, 3, 'full');
    nu_i{i} = sdpvar(1, 1, 'full');
    rho_i{i} = sdpvar(1, 1, 'full'); % Representing: rho
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

%% Combine constraints over all DGs (66a-Part1, 66b,66d,66e)

for i = 1:1:numOfDGs

    Ai = DG{i}.A;
    Bi = DG{i}.B;

    tagName = ['gammaTilde_',num2str(i)];
    constraintTags{end+1} = tagName;
    con0 = tag(gammaTilde_i{i} >= 0, tagName);

    % Constraint (66a-Part1)
    tagName = ['P_',num2str(i)];
    constraintTags{end+1} = tagName;
    con1 = tag(P_i{i} >= 0, tagName);
    
    % Constraint (66b)
    DMat = rhoTilde_i{i} * eye(3);
    MMat = [P_i{i}, zeros(3)];
    ThetaMat = [-Ai*P_i{i} - P_i{i}'*Ai' - Bi*K_i{i} - K_i{i}'*Bi', -eye(3) + 0.5*P_i{i};
                -eye(3) + 0.5*P_i{i}, -nu_i{i}*eye(3)];
    W = [DMat, MMat; MMat', ThetaMat];
    
    tagName = ['W_',num2str(i)];
    constraintTags{end+1} = tagName;
    con2 = tag(W >= 0, tagName);
    
    % Constraint (66d)
    p_i{i} = piVals(i); % predefined value

    tagName = ['nu_',num2str(i),'_low'];
    constraintTags{end+1} = tagName;
    con3_1 = tag(nu_i{i} >= -gammaTilde_i{i}/p_i{i}, tagName);

    tagName = ['nu_',num2str(i),'_high'];
    constraintTags{end+1} = tagName;
    con3_2 = tag(nu_i{i} <= 0, tagName);
    

    % Constraint (66e)
    tagName = ['rhoTilde_',num2str(i),'_low'];
    constraintTags{end+1} = tagName;
    con4_1 = tag(rhoTilde_i{i} >= 0, tagName);

    tagName = ['rhoTilde_',num2str(i),'_high1'];
    constraintTags{end+1} = tagName;
    con4_21 = tag(rhoTilde_i{i} <= p_i{i}, tagName);
    
    tagName = ['rhoTilde_',num2str(i),'_low'];
    constraintTags{end+1} = tagName;
    con4_22 = tag(rhoTilde_i{i} <= 4*gammaTilde_i{i}/p_i{i}, tagName);

    % New con8:
    % con4_31 = rho_i{i} >= 1/max(p_i{i}, 4*BarGamma/p_i{i});
    % con4_32 = rho_i{i} <= 100; %1/epsilon)
    % con4_33 = [rho_i{i}, 1; 1, rhoTilde_i{i}] >= 0;
    
    % Collecting Constraints
    constraints = [constraints, con0, con1, con2, con3_1, con3_2, con4_1, con4_21, con4_22];
    % constraints = [constraints, con0, con1, con2, con3_1, con3_2, con4_1, con4_21, con4_22, con4_31, con4_32, con4_33];
 end


%% Combine constraints over all Lines (66a-Part2, 66c)
for l = 1:1:numOfLines

    Rl = Line{l}.R;
    Ll = Line{l}.L;
    
    % Constraint (66a-Part2)

    tagName = ['PBar_',num2str(l)];
    constraintTags{end+1} = tagName;
    con5 = tag(P_l{l} >= 0, tagName);
    
    p_l{l} = plVals(l); %1/numOfLines;  % predefined value

    
    % Constraint (66c)
    W = [(2*P_l{l}*Rl)/Ll - rho_l{l}, -P_l{l}/Ll + 1/2;
         -P_l{l}/Ll + 1/2, -nu_l{l}];

    tagName = ['WBar_',num2str(l)];
    constraintTags{end+1} = tagName;
    con6 = tag(W >= 0, tagName);
    
    tagName = ['nuBar_',num2str(l),'_high'];
    constraintTags{end+1} = tagName;
    con7_0 = tag(nu_l{l} <= 0, tagName);

    tagName = ['rhoBar_',num2str(l),'low'];
    constraintTags{end+1} = tagName;
    con7_1 = tag(rho_l{l} >= 0, tagName);

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
           
           tagName = ['rhoBar_',num2str(l),'_low2_',num2str(i)];
           constraintTags{end+1} = tagName;
           con7_3 = tag(rho_l{l} >= ((rhoTilde_i{i})/(p_i{i}*p_l{l}))*((p_i{i}/(2*Ct))-(p_l{l}/2))^2, tagName);

           % con7_2Test{i,l} = rho_l{l} + (p_i{i}*nu_i{i})/(p_l{l}*Ct^2);
                
           % Constraint (66g)
           % epsilon = 0.001; % Minimum value
           n = 1;          % Number of intervals
           % rho_min = epsilon;
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
               
               con8 = [con8, con8_k];
               
               % Compute tilde_rho_i^{k-1} and tilde_y_i^{k-1}
               tilde_rho_i_prev = tilde_rho_i_k;
               tilde_y_i_prev = tilde_y_i_k;

           end

           %  % New con8:
           % con8 = nu_l{l} >= -p_i{i}*rho_i{i}/p_l{l};

           % Collecting Constraints  
          % constraints = [constraints, con7_2];
           % constraints = [constraints, con7_2, con7_3];
           constraints = [constraints, con7_2, con7_3, con8];
           % constraints = [constraints];

        end
    end
end



%% Solve the LMI problem (66)

costGamma = 0;
for  i = 1:numOfDGs
    % costGamma = costGamma + gammaTilde_i{i};
    % new con8
    costGamma = costGamma + (-nu_i{i}+rhoTilde_i{i}) + gammaTilde_i{i};
    % costGamma = costGamma + (-nu_i{i}+rhoTilde_i{i}) + gammaTilde_i{i} - 1000*rho_i{i};
end
for l = 1:numOfLines
    costGamma = costGamma + (-nu_l{l}-rho_l{l});
end

% Defining costfunction
costFunction = 1*costGamma; % Play with this choice

solverOptions = sdpsettings('solver', 'mosek', 'verbose', 0, 'debug', 0);

sol = optimize(constraints, costFunction, solverOptions);

statusLocalController = sol.problem == 0;

%% Display violated constraints by tag
% Check feasibility
feasibility = check(constraints);
% Display violated constraints by tag
for i = 1:length(feasibility)
    if feasibility(i) < -1e-6
        disp(['Constraint "', constraintTags{i}, '" is violated by',num2str(feasibility(i)),' .']);
    end
end


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

% for i = 1:1:numOfDGs
%     for l = 1:1:numOfLines
%         if B_il(i,l) ~= 0
%             con7_2_il = value(con7_2Test{i,l});
%             disp(['Con 7_2_',num2str(i),num2str(l),'=',num2str(con7_2_il)])
%         end
%     end
% end
% 
% for i = 1:1:numOfDGs
%     for l = 1:1:numOfLines
%         if B_il(i,l) ~= 0
%             con8_il = value(con8Test{i,l});
%             disp(['Con 8_',num2str(i),num2str(l),'=',num2str(con8_il)])
%         end
%     end
% end




end

