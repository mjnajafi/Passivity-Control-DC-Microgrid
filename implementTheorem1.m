% function [Vr, Is, statusThm1] = implementTheorem1(DG, Line, B_il)
function [Vr, Is, statusThm1] = implementTheorem1(DG, Line, B_il)
    numDGs = length(DG);
    numLines = length(Line);
    
   
    % Create matrices
    Rt = zeros(numDGs, numDGs);
    YL = zeros(numDGs, numDGs);
    Pn = zeros(numDGs, 1);  % Changed to vector
    IL_bar = zeros(numDGs, 1);
    V_nominal = zeros(numDGs, 1);
    
    % Extract parameters
    for i = 1:numDGs
        Rt(i,i) = DG{i}.R;
        YL(i,i) = DG{i}.Y;
        V_nominal(i) = DG{i}.refVoltage;
        % Pn(i) = DG{i}.Pn;  % Use power rating directly
        % Pn(i) = DG{i}.ratedCurrent;    % CORRECT - this is in Amperes
        Pn(i) = DG{i}.powerRating / DG{i}.refVoltage;  % Convert to current capacity
        IL_bar(i) = DG{i}.IL;
    end

    % System parameters
    Vr = sdpvar(numDGs, 1);
    Is = sdpvar(1, 1);
    % Vr = V_nominal;  % Fixed, not a variable

    
    % Build R matrix for lines
    R = zeros(numLines, numLines);
    for l = 1:numLines
        R(l,l) = Line{l}.R;
    end
    
    % Voltage bounds
    voltage_deviation_percent = 0.1;
    Vmin = (1 - voltage_deviation_percent/100) * V_nominal;
    Vmax = (1 + voltage_deviation_percent/100) * V_nominal;
    
    epsilon = 1e-6;
    
    % Create constraints
    constraints = [];
    
    % Main constraint from equation (53) in the paper
    constraints = [constraints, norm(Pn*Is - (B_il*inv(R)*B_il' + YL)*Vr - IL_bar) <= epsilon];
    
    % Bounds
    constraints = [constraints, Vmin <= Vr <= Vmax];
    constraints = [constraints, 0 <= Is <= 1];
    
    % Objective
    alphaV = 0;
    alphaI = 0;
    objective = alphaV * norm(Vr - V_nominal, 2)^2 + alphaI * Is;
    
    % Solve
    options = sdpsettings('solver', 'mosek', 'verbose', 1);
    sol = optimize(constraints, objective, options);
    
    statusThm1 = sol.problem == 0;
    
    if statusThm1
        Vr = value(Vr);
        Is = value(Is);
        
        % Debug output
        fprintf('\n==== Theorem 1 Solution ====\n');
        fprintf('Current sharing coefficient Is: %.4f\n', Is);
        
        % Check the actual currents
        currents = (B_il*inv(R)*B_il' + YL)*Vr + IL_bar;
        fprintf('\nActual currents:\n');
        for i = 1:numDGs
            fprintf('  DG%d: %.2f A (Pn*Is = %.2f A)\n', i, currents(i), Pn(i)*Is);
        end
        
        % Show load resistance effect
        for i = 1:numDGs
            fprintf('  DG%d: RL = %.2f Ω, YL = %.4f S\n', i, 1/DG{i}.Y, DG{i}.Y);
        end
    end
end