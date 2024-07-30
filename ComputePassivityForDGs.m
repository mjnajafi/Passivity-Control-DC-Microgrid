function [PVal, KVal, LVal, nuVal, rhoVal, status] = ComputePassivityForDGs(DG)
    
    
    A_DG = DG.A;
    B_DG = DG.B;


    solverOptions = sdpsettings('solver', 'mosek', 'verbose', 0);
    P = sdpvar(3, 3, 'symmetric');
    K = sdpvar(1, 3, 'full');
    nu = sdpvar(1, 1, 'full');
    rhoTilde = sdpvar(1, 1, 'full'); % Representing: 1/rho
    
    % Equation (66a)
    con1 = P >= 0;
    
    % Equation (66b)
    DMat = rhoTilde * eye(3);
    MMat = [P, zeros(3)];
    ThetaMat = [-A_DG*P - P'*A_DG' - B_DG*K - K'*B_DG', -eye(3) + 0.5*P; -eye(3) + 0.5*P, -nu*eye(3)];
    W = [DMat, MMat; MMat', ThetaMat];


    % Defined Constraints 
    con2 = W >= 0;

    con3_1 = -100 <= nu 
    con3_2 = nu <= 100;
    con4 = -100 <= rhoTilde & rhoTilde <= 100;

    % Total Constraints
    constraints = [con1, con2, con3, con4];

    % CostFunction - I do not know which cost function should be used!!!

    % costFunction = 0.0000001*(-nu + rhoTilde);
    costFunction = 1*(nu - rhoTilde);

    % Solution
    sol = optimize(constraints, costFunction, solverOptions);
    status = sol.problem == 0;

    % Extract optimal values
    PVal = value(P);
    KVal = value(K);
    LVal = KVal / PVal;
    nuVal = value(nu);
    rhoVal = 1 / value(rhoTilde);
   end
