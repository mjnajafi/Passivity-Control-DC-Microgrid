clc;
% clear;
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

    % Add constraints

    % Equation (66a)
    constraints = [constraints, PBar{line} >= 0];

       
    % Equation (66c)
    W = [(2*PBar{line}*Rl{line})/Ll{line} - rhoBar{line}, -PBar{line}/Ll{line} + 1/2;
         -PBar{line}/Ll{line} + 1/2, -nuBar{line}];
    
    pBarVal = 1;

    constraints = [constraints, W >= 0];

    con3 = rhoTilde{dg} <= pVal;
    con4 = nu{dg} <= 0;

    constraints = [constraints, con3];
    constraints = [constraints, con4];

    rhoBar1 = -(pVal*nu{dg})/(pBarVal*initial_C^2);
    rhoBar2 = (rhoTilde{dg})/(pVal*pBarVal)*((pVal/(2*initial_C))-(pBarVal/2))^2;

    con5 = rhoBar{line} >= rhoBar1;
    con6 = rhoBar{line} >= rhoBar2;

    constraints = [constraints, con5];
    constraints = [constraints, con6];
    
end

costFunction = sum([nuBar{:}] - [rhoBar{:}]);

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
