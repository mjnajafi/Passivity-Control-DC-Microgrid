

% First Code - Generating 4 A matrices
numDGs = 4;

% Initial parameter values for dgs
initial_R = 0.01; 
initial_L = 1e-2;  
initial_C = 2.2e-3;  
initial_Y = 0.2;  

% Initialize cell arrays to store results
PValues = cell(1, numDGs);
KValues = cell(1, numDGs);
LValues = cell(1, numDGs);
nuValues = cell(1, numDGs);
rhoValues = cell(1, numDGs);

% Loop through each DG
for dg = 1:1:numDGs
    % Add slight variations to the initial values
    R_values(dg) = initial_R + rand();
    L_values(dg) = initial_L + rand();
    C_values(dg) = initial_C + rand();
    Y_values(dg) = initial_Y + rand();

    A{dg} = [-Y_values(dg)/C_values(dg), 0, 1/C_values(dg); 
             -1/L_values(dg), -R_values(dg)/L_values(dg), 1/L_values(dg);
              1, 0, 0];
    B{dg} = [1; 1/L_values(dg); 0];
end


% Loop through each A matrix
for dg = 1:numDGs
    % Extract A and B matrices
    AMatrix = A{dg};
    BMatrix = B{dg};

    % Second Code (function) for each A matrix
    [PVal, KVal, LVal, nuVal, rhoVal] = ComputeParametersForDGs(AMatrix, BMatrix);

    % Store results
    PValues{dg} = PVal;
    KValues{dg} = KVal;
    LValues{dg} = LVal;
    nuValues{dg} = nuVal;
    rhoValues{dg} = rhoVal;

    % Display results for each A matrix
    disp(['Results for A matrix ', num2str(dg), ':']);
    disp('Optimal P for DG:');
    disp(PVal);
    disp('Optimal K for DG:');
    disp(KVal);
    disp('Optimal L (Local Controller Gains) for DG:');
    disp(LVal);
    disp('Optimal nu for DG:');
    disp(nuVal);
    disp('Optimal rho for DG:');
    disp(rhoVal);
    disp('--------------------------');
end


% Additional processing or analysis using the obtained results
% ...

% Function to compute parameters for DGs
function [PVal, KVal, LVal, nuVal, rhoVal] = ComputeParametersForDGs(A, B)
    % Constants for constraints
    nuBar = -40;
    rhoBar = 15;
    rhoBarBar = 1/rhoBar;

    % Constraints for nuHat, rhoHat, and global design
    nuHat = -0.01;
    rhoHat = 0.01;
    rhoHatBar = 1/rhoHat;
    gammaSqBar = 10;

    % Set up the LMI problem
    solverOptions = sdpsettings('solver', 'mosek', 'verbose', 0);
    P = sdpvar(3, 3, 'symmetric');
    K = sdpvar(1, 3, 'full');
    rhoTilde = sdpvar(1, 1, 'full'); % Representing: 1/rho
    nu = sdpvar(1, 1, 'full');

    % Basic Constraints
    con1 = P >= 0;

    % Approach 4 with rho = prespecified, nu < 0, and nu is free to maximize
    DMat = [rhoTilde * eye(3)];
    MMat = [P, zeros(3)];
    ThetaMat = [-A*P - P*A' - B*K - K'*B', -eye(3) + 0.5*P; -eye(3) + 0.5*P, -nu*eye(3)];
    W = [DMat, MMat; MMat', ThetaMat];
    con3 = W >= 0;

    % Modesty constraints on resulting nu and rho from the local design
    con4 = nu >= nuBar;
    con5 = rhoTilde >= rhoBarBar;

    % Global design constraints
    con8 = nu >= -gammaSqBar/(1/5); % nu >= -50
    con9 = rhoTilde <= 4*gammaSqBar/(1/5); % rhoBar >= 200

    % Total Cost and Constraints
    constraints = [con1, con3, con4, con5, con8, con9];
    costFunction = 0.0000001*(-nu + rhoTilde);

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

