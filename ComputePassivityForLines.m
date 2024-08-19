% This function, ComputePassivityForLines, calculates the passivity indices for a 
% transmission line based on its electrical parameters. The function formulates 
% and solves an optimization problem using Linear Matrix Inequality (LMI) methods 
% to find the passivity parameters PBar, nuBar, and rhoBar for the given line.

function [PBarVal, nuBarVal, rhoBarVal, status] = ComputePassivityForLines(Line)
    
    Ll = Line.L;
    Rl = Line.R;

    solverOptions = sdpsettings('solver', 'mosek', 'verbose', 0);
    PBar = sdpvar(1, 1, 'symmetric');
    nuBar = sdpvar(1, 1, 'full');
    rhoBar = sdpvar(1, 1, 'full'); 
    
    % Equation (66a)
    con1 = PBar >= 0;
    
    % Equation (66c)
    W = [(2*PBar*Rl)/Ll-rhoBar, -PBar/Ll+1/2;
            -PBar/Ll+1/2, -nuBar];


    % Defining the boundaries
    con2 = W >= 0;

    nu_bound = 100;
    rhoTilde_bound = 100;

    con3_1 = -nu_bound <= nuBar; 
    con3_2 = nuBar <= nu_bound;
    con4_1 = -rhoTilde_bound <= rhoBar; 
    con4_2 = rhoBar <= rhoTilde_bound;

    % Total Constraints
    constraints = [con1, con2, con3_1, con3_2, con4_1, con4_2];

   
    % costFunction = 0.0000001*(-nu + rhoTilde);
    costFunction = 1*(-nuBar - rhoBar);

    % Solution
    sol = optimize(constraints, costFunction, solverOptions);

    % If the LMI problem is feasible then the status is 1, otherwise 0.
    status = sol.problem == 0;

    % Extract optimal values
    PBarVal = value(PBar);
    nuBarVal = value(nuBar);
    rhoBarVal = value(rhoBar);
end
