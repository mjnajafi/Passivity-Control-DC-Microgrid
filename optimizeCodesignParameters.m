function [piVals, plVals] = optimizeCodesignParameters(DGs,Lines)
    % pARAMETERS
    options = optimoptions('fmincon');

    % Set OptimalityTolerance to 1e-3
    options = optimoptions(options, 'OptimalityTolerance', 1e-3); 

    % Set the Display option to 'iter' and StepTolerance to 1e-4
    options.Display = 'iter';
    options.StepTolerance = 1e-4;

    A = [];
    b = [];
    Aeq = [];
    beq = [];
    lb = zeros(N,1);
    ub = inf*ones(N,1);
    
    % p0 = (1/N)*ones(N,1); % initial condition
    piVals0 = (1/numOfDGs)*ones(1,numOfDGs); 
    plVals0 = (1/numOfLines)*ones(1,numOfLines); 
    pVals0 = [piVals0,plVals0];
    

    % Comment: Make variables like DGs and Lines global so that you have
    % access to them when evaluating f and nonlcon

    % f: Function of pVals parameters (each p_i and p_l used for local controller design), which will determine the resulting gamma value from the global design
    % f is what we need to optimize with respect to the used pVals parameters
    f = @(pVals)centralizedCodesign(pVals);
    
    % We need to optimize f while both local and global designs being
    % feasible. This is considered as a nonlinear constraint
    nonlcon = @(pVals)centralizedCodesignFeasibility(pVals);

    % Optimization 
    [pVals,fval] = fmincon(f,pVals0,A,b,Aeq,beq,lb,ub,nonlcon,options)

    % Optimum parameters
    piVals = pVals(1:numOfDGs);
    plVals = pVals((numOfDGs+1):end);
        
end

