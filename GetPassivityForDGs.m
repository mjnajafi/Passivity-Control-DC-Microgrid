clc;
clear;
close all;

% Number of Distributed Generators (DGs)
numDGs = 4;

% Initial parameter values for DGs
initial_R = 0.02; 
initial_L = 0.01;  
initial_C = 0.0022;  
initial_Load = 0.3;
initial_Y = 1/initial_Load;  

% Initialize cell arrays to store results
PValues = cell(1, numDGs);
KValues = cell(1, numDGs);
LValues = cell(1, numDGs);
nuValues = cell(1, numDGs);
rhoValues = cell(1, numDGs);
statusValues = cell(1, numDGs); 


% Loop through each DG
for dg = 1:numDGs
    % Add slight variations to the initial values
    R_values(dg) = initial_R + 0.01*rand();
    L_values(dg) = initial_L + 0.01*rand();
    C_values(dg) = initial_C + 0.001*rand();
    Y_values(dg) = initial_Y + 0.1*rand();

    A_DG{dg} = [-Y_values(dg)/C_values(dg), 1/C_values(dg), 0; 
             -1/L_values(dg), -R_values(dg)/L_values(dg), 0;
              1, 0, 0];
    B_DG{dg} = [0; 1/L_values(dg); 0];

    
    % Compute parameters for DGs
    [PVal, KVal, LVal, nuVal, rhoVal, status] = ComputePassivityForDGs(A_DG{dg}, B_DG{dg});


    % Store results
    PValues{dg} = PVal;
    KValues{dg} = KVal;
    LValues{dg} = LVal;
    nuValues{dg} = nuVal;
    rhoValues{dg} = rhoVal;
    statusValues{dg} = status;

    % Display results for each DG
    disp(['P Values for DG #', num2str(dg), ':']);
    disp(PVal);
    disp(['nu Values for DG #', num2str(dg), ':']);
    disp(nuVal);
    disp(['rho Values for DG #', num2str(dg), ':']);
    disp(rhoVal);
    disp(['L Values (Local Controller Gains) for DG #', num2str(dg), ':']);
    disp(LVal);
    disp('-------------------------------');
end

% Function to compute Passivity Indices for DGs
function [PVal, KVal, LVal, nuVal, rhoVal, status] = ComputePassivityForDGs(A_DG, B_DG)
    
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

    con3 = -100 <= nu & nu <= 100;
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
