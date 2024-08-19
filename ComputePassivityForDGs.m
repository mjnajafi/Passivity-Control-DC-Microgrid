% This function, ComputePassivityForDGs, computes the passivity indices for a given 
% Distributed Generator (DG) using a Linear Matrix Inequality (LMI) approach. The function 
% takes the state-space matrices of the DG (stored in the DG structure) and solves an 
% optimization problem to determine the passivity parameters P, K, nu, and rho. 

function [PVal, KVal, LVal, nuVal, rhoVal, status] = ComputePassivityForDGs(DG)
    
    
    Ai = DG.A;
    Bi = DG.B;


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
    ThetaMat = [-Ai*P - P'*Ai' - Bi*K - K'*Bi', -eye(3) + 0.5*P; -eye(3) + 0.5*P, -nu*eye(3)];
    W = [DMat, MMat; MMat', ThetaMat];


    % Defined Constraints 
    con2 = W >= 0;


    % Defining the boundaries
    nu_bound = 100;
    rhoTilde_bound = 100;

    con3_1 = -nu_bound <= nu;
    con3_2 = nu <= nu_bound;
    con4_1 = -rhoTilde_bound <= rhoTilde;
    con4_2 =  rhoTilde <= rhoTilde_bound;

    % Total Constraints
    constraints = [con1, con2, con3_1, con3_2, con4_1, con4_2];

    % costFunction = 0.0000001*(-nu + rhoTilde);
    costFunction = 1*(-nu + rhoTilde);

    % Solution
    sol = optimize(constraints, costFunction, solverOptions);

    % If the LMI problem is feasible then the status is 1, otherwise 0.
    status = sol.problem == 0;

    % Extract optimal values
    PVal = value(P);
    KVal = value(K);
    LVal = KVal / PVal;
    nuVal = value(nu);
    rhoVal = 1 / value(rhoTilde);

end
