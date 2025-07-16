% DC Microgrid Feasibility Analysis with Simplified Constraints
% Define system parameters for 4 DGs
clc;
clear all;
close all;

rng(7)

% System dimensions
N = 4;  % Number of DGs
L = 4;  % Number of transmission lines (example topology)

% DG Rated Current (arbitrary values)
% P_n = diag([800, 850, 820, 880]);  % Rated Current
% P_n = diag([0.9, 1, 0.8, 1]);  % Rated Current
P_n = diag([821/48, 414/48, 634/48, 1098/48]);  % Rated Current
% P_n = diag([900/48, 1000/48, 800/48, 1000/48]);  % Rated Current


% Load conductance matrix (arbitrary values)
% Y_L = diag([0.1, 0.15, 0.12, 0.18]);  % Conductance in Siemens
% Y_L = diag([DG{1}.Y, DG{2}.Y, DG{3}.Y, DG{4}.Y]);  % Conductance in Siemens
Y_L = diag([0.1783, 0.2039, 0.1962, 0.2026]);  % Conductance in Siemens

% Load current vector (arbitrary values)
% I_L = [10; 15; 12; 18];  % Load currents in Amperes
% I_L = [DG{1}.IL; DG{1}.IL; DG{1}.IL; DG{1}.IL];  % Load currents in Amperes
I_L = [4.7619; 5.8624; 5.8183; 4.4097];  % Load currents in Amperes


% Line resistance matrix (arbitrary values)
% R = diag([0.1, 0.12, 0.15, 0.11, 0.13]);  % Line resistances in Ohms
% R = diag([Line{1}.R, Line{2}.R, Line{3}.R, Line{4}.R]);  % Line resistances in Ohms
R = diag([0.0195, 0.0195, 0.0211, 0.0203]);  % Line resistances in Ohms

% Incidence matrix B
% B = [1  -1   0   0   0;
%      -1  1   1   0   0;
%      0   0  -1   1  -1;
%      0   0   0  -1   1];

B = [1  -0   0   0;
     0  1   1   0;
     -1   -1  0   1;
     0   0   -1  -1];

% Voltage constraints
V_min = 45.6;  % Minimum voltage (V)
V_max = 50.6;  % Maximum voltage (V)

% Calculate system matrices
BR = B * inv(R);
BRBt = BR * B';

% Set up optimization problem using YALMIP
yalmip('clear');

% Decision variables
V_r = sdpvar(N, 1);  % Reference voltage vector
I_s = sdpvar(1, 1);  % Current sharing index


% Constraints
Constraints = [];

% Equality constraint from current sharing
% Constraints = [Constraints; P_n*ones(N,1)*I_s == (BRBt + Y_L)*V_r + I_L];
epsilon = 1e-6;
Constraints = [Constraints; norm(P_n*ones(N,1)*I_s - (BRBt + Y_L)*V_r - I_L) <= epsilon];

% Simple voltage bounds
Constraints = [Constraints; V_min <= V_r <= V_max];

% Simple current sharing index bounds
Constraints = [Constraints; 0 <= I_s <= 1];

% Solve the feasibility problem
options = sdpsettings('verbose', 1, 'solver', 'sedumi');
sol = optimize(Constraints, [], options);

% Check and display results
if sol.problem == 0
    V_r_val = value(V_r);
    I_s_val = value(I_s);
    
    fprintf('Feasible solution found:\n\n');
    fprintf('Reference Voltages (V):\n');
    for i = 1:N
        fprintf('V_r%d = %.2f V\n', i, V_r_val(i));
    end
    fprintf('\nCurrent Sharing Index: %.4f\n', I_s_val);
    
    % Verify the equality constraint
    LHS = P_n*ones(N,1)*I_s_val;
    RHS = (BRBt + Y_L)*V_r_val + I_L;
    error = norm(LHS - RHS);
    fprintf('\nEquality Constraint Error: %.8f\n', error);
    
    % Calculate and display DG currents
    % I_dg = P_n*ones(N,1)*I_s_val;
    fprintf('\nResulting DG currents:\n');
    for i = 1:N
        fprintf('I_dg%d = %.2f A\n', i, I_dg(i));
    end
else
    fprintf('No feasible solution found\n');
    fprintf('Solver message: %s\n', sol.info);
end