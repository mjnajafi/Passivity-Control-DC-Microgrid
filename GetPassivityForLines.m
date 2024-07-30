%% Delete this

clc;
clear;
close all;

% Number of Distributed Generators (DGs)
numLines = 4;

% Initial parameter values for Lines
initial_Ll = 0.01;  
initial_Rl = 0.02;  

% Initialize cell arrays to store results
PBarValues = cell(1, numLines);
nuBarValues = cell(1, numLines); 
rhoBarValues = cell(1, numLines); 
statusValues = cell(1, numLines); 

% Loop through each DG
for line = 1:numLines
    
    Ll{line} = initial_Ll + 0.01*rand();
    Rl{line} = initial_Rl + 0.01*rand();

    
    % Compute parameters for DGs
    [PBarVal, nuBarVal, rhoBarVal, status] = ComputePassivityForLines(Ll{line}, Rl{line});

    % Store results
    PBarValues{line} = PBarVal;
    nuBarValues{line} = nuBarVal;
    rhoBarValues{line} = rhoBarVal;
    statusValues{line} = status;

    % Display results for each DG
    disp(['PBar Values for Line #', num2str(line), ':']);
    disp(PBarVal);
    disp(['nuBar Values for Line #', num2str(line), ':']);
    disp(nuBarVal);
    disp(['rhoBar Values for Line #', num2str(line), ':']);
    disp(rhoBarVal);
    disp('-------------------------------');
end

% Function to compute Passivity Indices for DGs
function [PBarVal, nuBarVal, rhoBarVal, status] = ComputePassivityForLines(Ll, Rl)
    
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
