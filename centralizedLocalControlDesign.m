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
%% Combine constraints over all DGs (66a-Part1, 66b,66d,66e)

for i = 1:1:numOfDGs

    Ai = DG{i}.A;
    Bi = DG{i}.B;

    con0 = gammaTilde_i{i} >=0;

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
    p_i{i} = piVals(i); % predefined value
    
    con3_1 = -gammaTilde_i{i}/p_i{i} <= nu_i{i};
    con3_2 = nu_i{i} <= 0;
    

    % Constraint (66e)
    con4_1 = 0 <= rhoTilde_i{i};

    con4_21 = rhoTilde_i{i} <= p_i{i};
    con4_22 = rhoTilde_i{i} <= 4*gammaTilde_i{i}/p_i{i};
    
    % Collecting Constraints
    constraints = [constraints, con0, con1, con2, con3_1, con3_2, con4_1, con4_21, con4_22];
 end


%% Combine constraints over all Lines (66a-Part2, 66c)
for l = 1:1:numOfLines

    Rl = Line{l}.R;
    Ll = Line{l}.L;
    
    % Constraint (66a-Part2)
    con5 = P_l{l} >= 0;

    % 
    con7_0 = nu_l{l} <= 0; % See Constraint (51)
    con7_1 = rho_l{l} >= 0; % See Constraint (53)
    
    p_l{l} = plVals(l); %1/numOfLines;  % predefined value

    
    % Constraint (66c)
    Z = [(2*P_l{l}*Rl)/Ll - rho_l{l}, -P_l{l}/Ll + 1/2;
         -P_l{l}/Ll + 1/2, -nu_l{l}];

    con6 = Z >= 0;
     
   

    % Collecting Constraints
    constraints = [constraints, con5, con6, con7_0, con7_1];
end


% %% Combine all mixed constraints (66f, 66g)
% %%%% Comment: Debug these constraints, why their addition affects the
% %%%% resulting passivity properties...?
% for i = 1:1:numOfDGs
%     for l = 1:1:numOfLines
% 
%         Ct = DG{i}.C;
% 
%         if B_il(i,l) ~= 0
% 
%            % Constraint (66f)
%            con7_2 = rho_l{l} >= -(p_i{i}*nu_i{i})/(p_l{l}*Ct^2);
%            con7_3 = rho_l{l} >= ((rhoTilde_i{i})/(p_i{i}*p_l{l}))*((p_i{i}/(2*Ct))-(p_l{l}/2))^2;
% 
% 
% 
%            % % Constraint (66g)
%            % epsilon = 0.001; % Minimum value
%            % n = 100; % Number of intervals
%            % % rho_min = epsilon;
%            % rho_max = min(p_i{i}, 4*BarGamma/p_i{i});
%            % rho_min = rho_max/1000;
%            % delta_i = (rho_max - rho_min) / n;
%            % 
%            % 
%            % % con8Test{i,l} = nu_l{l} +  p_i{i}/(rhoTilde_i{i}*p_l{l}); % This needs to be positive for the global controller to be feasible
%            % 
%            % 
%            % % % Initialize cell array to store individual constraints
%            % con8 = [];
%            % 
%            % % Loop over each k from 1 to n to create individual constraints
%            % tilde_rho_i_prev = rho_min;
%            % tilde_y_i_prev = -p_i{i} / (p_l{l} * tilde_rho_i_prev);
%            % 
%            % for k = 1:n
%            % 
%            %     % Compute tilde_rho_i^k
%            %    tilde_rho_i_k = rho_min + k * delta_i;   
%            % 
%            %     % Compute tilde_y_i^k
%            %     tilde_y_i_k = -p_i{i} / (p_l{l} * tilde_rho_i_k);
%            % 
%            %     % Compute m_k and c_k
%            %     m_k = (tilde_y_i_k - tilde_y_i_prev) / delta_i;
%            %     c_k = tilde_y_i_k - m_k * tilde_rho_i_k;
%            % 
%            %     % % Define Constraint (66g)
%            %     if ~isnan(m_k)
%            %          con8_k = nu_l{l} >= m_k * rhoTilde_i{i} + c_k;
%            %          con8 = [con8, con8_k];
%            %     else
%            %     %     i
%            %     %     l
%            %     %     k
%            %          % delta_i
%            %     %      m_k
%            %     % end
%            % 
%            % 
%            %     % Compute tilde_rho_i^{k-1} and tilde_y_i^{k-1}
%            %     tilde_rho_i_prev = tilde_rho_i_k;
%            %     tilde_y_i_prev = tilde_y_i_k;
%            % 
%            % % end
% 
%            % % Constraint (66g)
%            % epsilon = 0.001; % Minimum value
%            % rho_min = epsilon;
%            rho_max = min(p_i{i}, 4*BarGamma/p_i{i});
%            rho_min = rho_max/100;
%            num_breakpoints = 100; % Desired number of breakpoints
% 
%            % Generate breakpoints linearly spaced between rho_min and rho_max
%            rho_breakpoints = linspace(rho_min, rho_max, num_breakpoints);
% 
%            n = length(rho_breakpoints) - 1; % Number of segments
%            con8 = []; % Initialize constraint array
% 
%            % Loop through each segment defined by breakpoints
%            for j = 1:n
%                % Breakpoints
%                rho_left = rho_breakpoints(j);
%                rho_right = rho_breakpoints(j + 1);
% 
%                % Evaluate the function at breakpoints
%                nu_left = -p_i{i} / (p_l{l} * rho_left);
%                nu_right = -p_i{i} / (p_l{l} * rho_right);
% 
%                 % Calculate the slope (m) and intercept (c) for linear approximation
%                 m = (nu_right - nu_left) / (rho_right - rho_left);
%                 c = nu_left - m * rho_left;
% 
%                 % Create constraints for the current segment
%                 % For -p_i / (p_l * tilde_rho_i) < nu_l
%                 % => nu_l >= m * tilde_rho_i + c
%                 con8_k = nu_l{l} >= m * rhoTilde_i{i} + c;
% 
%                 % Append the constraints for the current segment
%                 con8 = [con8, con8_k];
%            end
% 
%            % Add upper bound constraint for nu_l
%            con8_upper = nu_l{l} <= 0;
%            con8 = [con8, con8_upper];
% 
%            % Display breakpoints for verification
%            disp('Breakpoints for tilde_rho_i:');
%            disp(rho_breakpoints);
% 
% 
%             % 
%             % % Generate breakpoints
%             % rho_breakpoints = linspace(rho_min, rho_max, num_breakpoints);
%             % 
%             % n = length(rho_breakpoints) - 1; % Number of segments
%             % con8 = []; % Initialize constraint array
%             % 
%             % % Loop through each segment defined by breakpoints
%             % for j = 1:n
%             %     % Breakpoints
%             %     rho_left = rho_breakpoints(j);
%             %     rho_right = rho_breakpoints(j + 1);
%             % 
%             %     % Evaluate the function at breakpoints
%             %     nu_left = -p_i / (p_l * rho_left);
%             %     nu_right = -p_i / (p_l * rho_right);
%             % 
%             %     % Calculate the average slope (m) for linear approximation
%             %     avg_nu = (nu_left + nu_right) / 2; % Average value for a smoother transition
%             %     m = (nu_right - nu_left) / (rho_right - rho_left); % Original slope for reference
%             %     c = avg_nu - m * ((rho_left + rho_right) / 2); % Calculate intercept based on average
%             % 
%             %     % Create relaxed constraints
%             %     % Allowing for some slack in the constraint
%             %     slack = 0.1; % You can adjust this value to relax the constraint
%             %     con8_k = nu_l{l} >= m * rhoTilde_i{i} + c - slack; % Relaxed lower bound
%             % 
%             %     % Append the constraints for the current segment
%             %     con8 = [con8, con8_k];
%             % end
%             % 
%             % % Add upper bound constraint for nu_l
%             % con8_upper = nu_l{l} <= 0;
%             % con8 = [con8, con8_upper];
%             % 
%             % % Display breakpoints for verification
%             % disp('Breakpoints for tilde_rho_i:');
%             % disp(rho_breakpoints);
% 
% 
% 
% 
% 
%            % Collecting Constraints  
%            constraints = [constraints, con7_2, con7_3, con8];
%         end
%     end
% end



%% Solve the LMI problem (66)

costGamma = 0;
for  i = 1:numOfDGs
    % costGamma = costGamma + gammaTilde_i{i};
    costGamma = costGamma + (-nu_i{i}+rhoTilde_i{i}) + gammaTilde_i{i};
end

% Defining costfunction
costFunction = 1*costGamma; % Play with this choice

solverOptions = sdpsettings('solver', 'mosek', 'verbose', 1, 'debug', 0);

sol = optimize(constraints, costFunction, solverOptions);

statusLocalController = sol.problem == 0;


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
%         con8_il = value(con8Test{i,l});
%         disp(['Con 8_',num2str(i),num2str(l),'=',num2str(con8_il)])
%     end
% end




end
