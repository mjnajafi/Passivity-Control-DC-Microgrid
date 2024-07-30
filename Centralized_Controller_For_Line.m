%% Delete this

clc;
clear;
close all;

% Number of Lines
numLines = 4;

% Initial parameter values for Lines
initial_Ll = 0.01;  
initial_Rl = 0.02;  

% Initialize matrices to store system parameters
Ll = cell(1, numLines);
Rl = cell(1, numLines);

% Generate L and R values for each line
for line = 1:numLines
    Ll{line} = initial_Ll + 0.01*rand();
    Rl{line} = initial_Rl + 0.01*rand();
end

% Decision variables for all lines
PBar = cell(1, numLines); 
nuBar = cell(1, numLines); 
rhoBar = cell(1, numLines); 

% Constraints
constraints = [];

for line = 1:numLines
    
    PBar{line} = sdpvar(1, 1, 'symmetric');
    nuBar{line} = sdpvar(1, 1, 'full');
    rhoBar{line} = sdpvar(1, 1, 'full'); 

    % Equation (66a)
    constraints = [constraints, PBar{line} >= 0];

    % Equation (66a)
    con1 = PBar{line} >= 0;
    
    % Equation (66c)
    W = [(2*PBar{line}*Rl{line})/Ll{line} - rhoBar{line}, -PBar{line}/Ll{line} + 1/2;
         -PBar{line}/Ll{line} + 1/2, -nuBar{line}];
    
    % Add constraints
    
    constraints = [constraints, W >= 0];
    constraints = [constraints, -100 <= nuBar{line} & nuBar{line} <= 100];
    constraints = [constraints, -100 <= rhoBar{line} & rhoBar{line} <= 100];
end

costFunction = sum(nuBar{line} - rhoBar{line});

solverOptions = sdpsettings('solver', 'mosek', 'verbose', 0);

sol = optimize(constraints, costFunction, solverOptions);

status = sol.problem == 0;

% Extract and display results
PBarValues = cell(1, numLines);
nuBarValues = cell(1, numLines);
rhoBarValues = cell(1, numLines);

for line = 1:numLines
    PBarVal = value(PBar{line});
    nuBarVal = value(nuBar{line});
    rhoBarVal = value(rhoBar{line});
    
    % Display results for each line
    disp(['PBar Values for Line #', num2str(line), ':']);
    disp(PBarVal);
    disp(['nuBar Values for Line #', num2str(line), ':']);
    disp(nuBarVal);
    disp(['rhoBar Values for Line #', num2str(line), ':']);
    disp(rhoBarVal);
    disp('-------------------------------');
end
