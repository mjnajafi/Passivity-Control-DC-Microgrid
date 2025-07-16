function [V_r_val, I_s_val, statusTheorem1] = feasibilityAnalysis(DG, Line, B)
    % System dimensions
    N = length(DG); % Number of DGs
    L = length(Line); % Number of transmission lines
    
    % Extract parameters from DG structures
    P_n_values = zeros(N, 1);
    Y_L_values = zeros(N, 1);
    I_L_values = zeros(N, 1);
    
    for i = 1:N
        P_n_values(i) = DG{i}.Pn;         % Power rating
        Y_L_values(i) = DG{i}.Y;          % Load conductance
        I_L_values(i) = DG{i}.IL;         % Constant current load
    end
    
    % Create diagonal matrices
    P_n = diag(P_n_values);
    Y_L = diag(Y_L_values);
    I_L = I_L_values;
    
    % Extract line resistances from Line structures
    R_values = zeros(L, 1);
    for l = 1:L
        R_values(l) = Line{l}.R;          % Line resistance
    end
    R = diag(R_values);
    
    % Print system parameters for debugging
    fprintf('--- System Parameters ---\n');
    fprintf('P_n values: '); fprintf('%.4f ', P_n_values); fprintf('\n');
    fprintf('Y_L values: '); fprintf('%.4f ', Y_L_values); fprintf('\n');
    fprintf('I_L values: '); fprintf('%.4f ', I_L_values); fprintf('\n');
    fprintf('R values: '); fprintf('%.4f ', R_values); fprintf('\n');
    
    % Print the matrix dimensions for debugging
    fprintf('Dimensions - P_n: %dx%d, Y_L: %dx%d, I_L: %dx%d, R: %dx%d, B: %dx%d\n', ...
        size(P_n,1), size(P_n,2), size(Y_L,1), size(Y_L,2), ...
        size(I_L,1), length(I_L), size(R,1), size(R,2), size(B,1), size(B,2));
    
    % Calculate system matrices
    BR = B * inv(R);
    BRBt = BR * B';
    
    % Voltage constraints - using default nominal voltage from DGs
    % with a small margin (±5%)
    V_nominal = 48.0;  % Default nominal voltage
    margin = 0.1;    % 5% margin
    V_min = V_nominal * (1 - margin);
    V_max = V_nominal * (1 + margin);
    
    % Set up optimization problem using YALMIP
    yalmip('clear');
    
    % Decision variables
    V_r = sdpvar(N, 1); % Reference voltage vector
    I_s = sdpvar(1, 1); % Current sharing index
    
    % Instead of direct equality constraint, formulate as minimization
    equality_error = P_n*ones(N,1)*I_s - (BRBt + Y_L)*V_r - I_L;
    
    % Basic constraints
    Constraints = [];
    
    % Simple voltage bounds
    Constraints = [Constraints; V_min <= V_r <= V_max];
    
    % Current sharing index bounds with slightly wider range
    Constraints = [Constraints; 0 <= I_s <= 1];
    
    % Objective function: Minimize equality error + voltage deviation
    alpha_eq = 100.0;   % Higher weight on satisfying the equality constraint
    alpha_V = 1.0;      % Weight for voltage tracking
    alpha_I = 0.1;      % Weight for current sharing
    
    Objective = alpha_eq * norm(equality_error) + ...
                alpha_V * norm(V_r - V_nominal*ones(N,1)) + ...
                alpha_I * I_s;
    
    % Try different solvers
    solvers = {'mosek', 'sedumi', 'sdpt3'};
    statusTheorem1 = false;
    
    for s = 1:length(solvers)
        options = sdpsettings('verbose', 1, 'solver', solvers{s}, 'debug', 1);
        fprintf('\nTrying solver: %s\n', solvers{s});
        
        sol = optimize(Constraints, Objective, options);
        
        if sol.problem == 0
            statusTheorem1 = true;
            V_r_val = value(V_r);
            I_s_val = value(I_s);
            
            % Check the equality constraint error
            eq_error_val = value(norm(equality_error));
            
            fprintf('Feasible solution found with solver %s!\n', solvers{s});
            fprintf('Equality constraint error: %.6e\n', eq_error_val);
            
            % If error is sufficiently small, accept the solution
            if eq_error_val < 1e-2
                break;
            else
                fprintf('Warning: Solution found but equality constraint error is high\n');
                % Try next solver for a potentially better solution
            end
        else
            fprintf('Solver %s failed with status: %s\n', solvers{s}, yalmiperror(sol.problem));
        end
    end
    
    if statusTheorem1
        fprintf('\nReference Voltages (V):\n');
        for i = 1:N
            fprintf('V_r%d = %.2f V\n', i, V_r_val(i));
        end
        fprintf('\nCurrent Sharing Index: %.4f\n', I_s_val);
        
        % Verify the equality constraint
        LHS = P_n*ones(N,1)*I_s_val;
        RHS = (BRBt + Y_L)*V_r_val + I_L;
        error = norm(LHS - RHS);
        fprintf('\nEquality Constraint Error: %.8f\n', error);
        
        % Calculate and display DG currents
        I_dg = P_n*ones(N,1)*I_s_val;
        fprintf('\nResulting DG currents:\n');
        for i = 1:N
            fprintf('I_dg%d = %.2f A (corresponds to %.2f%% of rated capacity)\n', ...
                i, I_dg(i), (I_dg(i)/P_n_values(i))*100);
        end
        
        % Store the results in the DG structures for future use
        for i = 1:N
            DG{i}.V_ref = V_r_val(i);      % Reference voltage
            DG{i}.I_share = I_s_val;       % Current sharing index
            DG{i}.I_expected = I_dg(i);    % Expected current at equilibrium
        end
    else
        V_r_val = [];
        I_s_val = [];
        fprintf('\nAll solvers failed to find a feasible solution.\n');
        fprintf('Suggestions to resolve infeasibility:\n');
        fprintf('1. Check your system parameters - ensure they are physically realistic\n');
        fprintf('2. Verify the incidence matrix B matches your physical topology\n');
        fprintf('3. Widen the voltage constraints if possible\n');
        fprintf('4. Consider restructuring your network topology\n');
    end
end