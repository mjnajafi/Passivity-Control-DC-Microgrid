%% Delete this

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

% Initialize matrices to store system parameters
A_DG = cell(1, numDGs);
B_DG = cell(1, numDGs);

% Generate A and B matrices for each DG
for dg = 1:numDGs
    R_values(dg) = initial_R + 0.01*rand();
    L_values(dg) = initial_L + 0.01*rand();
    C_values(dg) = initial_C + 0.001*rand();
    Y_values(dg) = initial_Y + 0.1*rand();

    A_DG{dg} = [-Y_values(dg)/C_values(dg), 1/C_values(dg), 0;
                -1/L_values(dg), -R_values(dg)/L_values(dg), 0;
                1, 0, 0];
    B_DG{dg} = [0; 1/L_values(dg); 0];
end

% Decision variables for all DGs
P = cell(1, numDGs);
K = cell(1, numDGs);
nu = cell(1,numDGs);
rhoTilde = cell(1,numDGs);

constraints = [];

for dg = 1:numDGs
    % 
    % P{dg} = sdpvar(3, 3, 'symmetric');
    % K{dg} = sdpvar(1, 3, 'full');
    % nu{dg} = sdpvar(1, 1, 'full');
    % rhoTilde{dg} = sdpvar(1, 1, 'full'); % Representing: 1/rho
    % 
    % % Equation (66a)
    % constraints = [constraints, P{dg} >= 0];
    % 
    % % Equation (66b)
    % DMat = rhoTilde{dg} * eye(3);
    % MMat = [P{dg}, zeros(3)];
    % ThetaMat = [-A_DG{dg}*P{dg} - P{dg}'*A_DG{dg}' - B_DG{dg}*K{dg} - K{dg}'*B_DG{dg}', -eye(3) + 0.5*P{dg};
    %             -eye(3) + 0.5*P{dg}, -nu{dg}*eye(3)];
    % W = [DMat, MMat; MMat', ThetaMat];
    % 
    % pVal = 1; % This value is predefined. 
    % 
    % constraints = [constraints, W >= 0];
    % 
    % 
    % con3 = rhoTilde{dg} <= pVal;
    % con4 = nu{dg} <= 0;
    % 
    % constraints = [constraints, con3];
    % constraints = [constraints, con4];

    %% Why not Call the function: ComputePassivityForDGs ... 
    [PVal, KVal, LVal, nuVal, rhoVal, status] = ComputePassivityForDGs(A_DG{DG}, B_DG{DG})
    % Display results for each DG
    disp(['P Values for DG #', num2str(dg), ':']);
    disp(PVal);
    disp(['nu Values for DG #', num2str(dg), ':']);
    disp(nuVal);
    disp(['rhoTilde Values for DG #', num2str(dg), ':']);
    disp(rhoTildeVal);
    disp(['rho Values for DG #', num2str(dg), ':']);
    disp(rhoVal);
    disp(['L Values (Local Controller Gains) for DG #', num2str(dg), ':']);
    disp(LVal);
    

end
% 
% solverOptions = sdpsettings('solver', 'mosek', 'verbose', 0);
% 
% costFunction = sum([nu{:}] - [rhoTilde{:}]);
% 
% sol = optimize(constraints, costFunction, solverOptions);
% 
% status = sol.problem == 0;
% 
% % Extract and display results
% for dg = 1:numDGs
%     PVal = value(P{dg});
%     KVal = value(K{dg});
%     LVal = KVal / PVal;
%     nuVal = value(nu{dg});
%     rhoTildeVal = value(rhoTilde{dg});
%     rhoVal = 1 / value(rhoTilde{dg});
% 
%     % Display results for each DG
%     disp(['P Values for DG #', num2str(dg), ':']);
%     disp(PVal);
%     disp(['nu Values for DG #', num2str(dg), ':']);
%     disp(nuVal);
%     disp(['rhoTilde Values for DG #', num2str(dg), ':']);
%     disp(rhoTildeVal);
%     disp(['rho Values for DG #', num2str(dg), ':']);
%     disp(rhoVal);
%     disp(['L Values (Local Controller Gains) for DG #', num2str(dg), ':']);
%     disp(LVal);
% 
%     disp('-------------------------------');
% end
