function [Vr, Is, statusThm1] = implementTheorem1_debug(DG, Line, B_il)
    numDGs = length(DG);
    numLines = length(Line);
    
    % Extract all parameters first
    Rt = zeros(numDGs, numDGs);
    YL = zeros(numDGs, numDGs);
    Pn = zeros(numDGs, 1);
    IL_bar = zeros(numDGs, 1);
    V_nominal = zeros(numDGs, 1);
    
    fprintf('\n=== DG Parameters ===\n');
    for i = 1:numDGs
        Rt(i,i) = DG{i}.R;
        YL(i,i) = DG{i}.Y;
        V_nominal(i) = DG{i}.refVoltage;
        Pn(i) = DG{i}.powerRating / DG{i}.refVoltage;
        IL_bar(i) = DG{i}.IL;
        
        fprintf('DG%d: RL=%.2f, YL=%.4f, Pn=%.2f, IL=%.2f, Vnom=%.2f\n', ...
            i, 1/DG{i}.Y, DG{i}.Y, Pn(i), IL_bar(i), V_nominal(i));
    end
    
    % Build R matrix
    R = zeros(numLines, numLines);
    for l = 1:numLines
        R(l,l) = Line{l}.R;
    end
    
    % Calculate the network admittance matrix
    Y_network = B_il * inv(R) * B_il';
    
    fprintf('\n=== Network Analysis ===\n');
    fprintf('Network admittance matrix:\n');
    disp(Y_network);
    fprintf('Total admittance (YL + Y_network):\n');
    disp(YL + Y_network);
    
    % Before optimization, let's check what Is should be theoretically
    % If we fix Vr = V_nominal, what Is do we need?
    current_at_nominal = (Y_network + YL) * V_nominal + IL_bar;
    Is_theoretical = sum(current_at_nominal) / sum(Pn);
    
    fprintf('\n=== Theoretical Analysis (at nominal voltage) ===\n');
    fprintf('Current at nominal voltage: %s\n', mat2str(current_at_nominal, 3));
    fprintf('Total current needed: %.2f A\n', sum(current_at_nominal));
    fprintf('Total capacity: %.2f A\n', sum(Pn));
    fprintf('Theoretical Is: %.4f\n', Is_theoretical);
    
    % Now check for different load values
    fprintf('\n=== Load Sensitivity Check ===\n');
    for scale = [0.5, 1.0, 2.0]
        YL_test = YL * scale;
        current_test = (Y_network + YL_test) * V_nominal + IL_bar;
        Is_test = sum(current_test) / sum(Pn);
        fprintf('YL scale %.1f: Total current=%.2f A, Is=%.4f\n', ...
            scale, sum(current_test), Is_test);
    end
    
    % Now do the actual optimization
    Vr = sdpvar(numDGs, 1);
    Is = sdpvar(1, 1);
    
    epsilon = 1e-6;
    constraints = [];
    
    % Main constraint
    constraints = [constraints, norm(Pn*Is - (Y_network + YL)*Vr - IL_bar) <= epsilon];
    
    % Bounds
    voltage_deviation_percent = 5;
    Vmin = (1 - voltage_deviation_percent/100) * V_nominal;
    Vmax = (1 + voltage_deviation_percent/100) * V_nominal;
    constraints = [constraints, Vmin <= Vr <= Vmax];
    constraints = [constraints, 0 <= Is <= 1];
    
    % Objective
    alphaV = 1;
    alphaI = 0.1;
    objective = alphaV * norm(Vr - V_nominal, 2)^2 + alphaI * Is;
    
    % Solve
    options = sdpsettings('solver', 'mosek', 'verbose', 1);
    sol = optimize(constraints, objective, options);
    
    statusThm1 = sol.problem == 0;
    
    if statusThm1
        Vr_val = value(Vr);
        Is_val = value(Is);
        
        fprintf('\n=== Optimization Results ===\n');
        fprintf('Is = %.6f\n', Is_val);
        fprintf('Voltage values and deviations:\n');
        for i = 1:numDGs
            deviation = (Vr_val(i) - V_nominal(i))/V_nominal(i) * 100;
            fprintf('  DG%d: Vr=%.3f V (%.2f%% from nominal)\n', i, Vr_val(i), deviation);
        end
        
        % Verify the constraint
        actual_current = (Y_network + YL)*Vr_val + IL_bar;
        target_current = Pn*Is_val;
        constraint_error = norm(target_current - actual_current);
        
        fprintf('\n=== Constraint Verification ===\n');
        fprintf('Actual current: %s\n', mat2str(actual_current, 3));
        fprintf('Target current: %s\n', mat2str(target_current, 3));
        fprintf('Constraint error: %.6e\n', constraint_error);
        
        % Component analysis
        fprintf('\n=== Current Components ===\n');
        load_current = YL * Vr_val;
        network_current = Y_network * Vr_val;
        const_current = IL_bar;
        
        fprintf('Load current (YL*Vr): %s\n', mat2str(load_current, 3));
        fprintf('Network current: %s\n', mat2str(network_current, 3));
        fprintf('Constant current: %s\n', mat2str(const_current, 3));
        
        % Return values
        Vr = Vr_val;
        Is = Is_val;
    else
        fprintf('Optimization failed: %s\n', yalmiperror(sol.problem));
        Vr = [];
        Is = [];
    end
end