clc;
close all;



% First Code - Generating 4 A matrices
numLines = 4;

% Initial parameter values for line
initial_Ll = 10e-6;  
initial_Rl = 0.01;  

% Initialize cell arrays to store results
PValues_Bar = cell(1, numLines);
KValues_Bar = cell(1, numLines);
LValues_Bar = cell(1, numLines);
nuValues_Bar = cell(1, numLines);
rhoValues_Bar = cell(1, numLines);


% Loop through each Line
for line = 1:1:numLines
    % Add slight variations to the initial values
    Ll_values(line) = initial_Ll + rand();
    Rl_values(line) = initial_Rl + rand();

    A{line}=[-Rl_values(line)/Ll_values(line)];
    B{line}=[1/Ll_values(line)];
end


for line = 1:numLines
    % Extract A and B matrices
    AMatrix = A{line};
    BMatrix = B{line};

    % Second Code (function) for each A matrix
    [PVal, KVal, LVal, nuVal, rhoVal] = ComputeParametersForLines(AMatrix, BMatrix);

    % Store results
    PValues_Bar{line} = PVal;
    KValues_Bar{line} = KVal;
    LValues_Bar{line} = LVal;
    nuValues_Bar{line} = nuVal;
    rhoValues_Bar{line} = rhoVal;

    % Display results for each A matrix
    disp(['Results for A matrix ', num2str(line), ':']);
    disp('Optimal P_Bar for line:');
    disp(PVal);
    disp('Optimal K_Bar for line:');
    disp(KVal);
    disp('Optimal L_Bar (Local Controller Gains) for line:');
    disp(LVal);
    disp('Optimal nu_Bar for line:');
    disp(nuVal);
    disp('Optimal rho_Bar for line:');
    disp(rhoVal);
    disp('--------------------------');
end




function [PVal, KVal, LVal, nuVal, rhoVal] = ComputeParametersForLines(A, B)
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
    P = sdpvar(1, 1, 'symmetric');
    K = sdpvar(1, 1, 'full');
    rhoTilde = sdpvar(1, 1, 'full'); % Representing: 1/rho
    nu = sdpvar(1, 1, 'full');

    % Basic Constraints
    con1 = P >= 0;

    % Approach 4 with rho = prespecified, nu < 0, and nu is free to maximize
    DMat = [rhoTilde * eye(1)];
    MMat = [P, zeros(1)];
    ThetaMat = [-A*P - P*A' - B*K - K'*B', -eye(1) + 0.5*P; -eye(1) + 0.5*P, -nu*eye(1)];
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
