function [PBarVal, nuBarVal, rhoBarVal, status] = ComputePassivityForLines(Line)
    
    Ll = Line.L;
    Rl = Line.R;

    solverOptions = sdpsettings('solver', 'mosek', 'verbose', 0);
    PBar = sdpvar(1, 1, 'symmetric');
    nuBar = sdpvar(1, 1, 'full');
    rhoBar = sdpvar(1, 1, 'full'); % Representing: 1/rho
    
    % Equation (66a)
    con1 = PBar >= 0;
    
    % Equation (66c)
    W = [(2*PBar*Rl)/Ll-rhoBar, -PBar/Ll+1/2;
            -PBar/Ll+1/2, -nuBar];


    % Defined Constraints 
    con2 = W >= 0;

    con3 = -100 <= nuBar & nuBar <= 100;
    con4 = -100 <= rhoBar & rhoBar <= 100;

    % Total Constraints
    constraints = [con1, con2, con3, con4];

    % CostFunction - I do not know which cost function should be used!!!

    % costFunction = 0.0000001*(-nu + rhoTilde);
    costFunction = 1*(nuBar - rhoBar);

    % Solution
    sol = optimize(constraints, costFunction, solverOptions);
    status = sol.problem == 0;

    % Extract optimal values
    PBarVal = value(PBar);
    nuBarVal = value(nuBar);
    rhoBarVal = value(rhoBar);
   end
