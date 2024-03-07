% Number of DGs and lines
numDGs = 4;
numLines = 4;

% Initialize the incidence matrix B with zeros
B = zeros(numDGs, numLines);

% Define the physical topology
topology = [1, 1; 2, 2; 3, 3; 4, 4]; % Assuming DG1 is connected to Line 1, DG2 to Line 2, and so on.

% Populate the incidence matrix based on the topology
for i = 1:size(topology, 1)
    DGU = topology(i, 1);
    line = topology(i, 2);
    
    % Assign +1 for out-neighbor and -1 for in-neighbor
    B(DGU, line) = 1;
    B(mod(DGU, numDGs) + 1, line) = -1;
end
% Display the incidence matrix
disp('Incidence Matrix (B):');
disp(B);

% Number of DGs
N = 4; 

 %creating the C Matrix. 
c_11=[B(1,1)/Ll_values(1), 0 , 0];
c_21=[B(2,1)/Ll_values(1), 0 , 0];
c_31=[B(3,1)/Ll_values(1), 0 , 0];
c_41=[B(4,1)/Ll_values(1), 0 , 0];

c_12=[B(1,2)/Ll_values(2), 0 , 0];
c_22=[B(2,2)/Ll_values(2), 0 , 0];
c_32=[B(3,2)/Ll_values(2), 0 , 0];
c_42=[B(4,2)/Ll_values(2), 0 , 0];

c_13=[B(1,3)/Ll_values(3), 0 , 0];
c_23=[B(2,3)/Ll_values(3), 0 , 0];
c_33=[B(3,3)/Ll_values(3), 0 , 0];
c_43=[B(4,3)/Ll_values(3), 0 , 0];

c_14=[B(1,4)/Ll_values(4), 0 , 0];
c_24=[B(2,4)/Ll_values(4), 0 , 0];
c_34=[B(3,4)/Ll_values(4), 0 , 0];
c_44=[B(4,4)/Ll_values(4), 0 , 0];

C=[c_11,c_12,c_13,c_14;
    c_21,c_22,c_23,c_24;
    c_31, c_32,c_33,c_34;
    c_41,c_42,c_43,c_44];

% In this part, I created a compact form.
% C = zeros(4, 12);
% 
% for i = 1:4
%     for j = 1:4
%         C(i, (j-1)*3 + 1:(j-1)*3 + 3) = [B(i, j) / Ll_values(j), 0, 0];
%     end
% end


%creating BarC Matrix:
Barc_11=[-B(1,1)/C_values(1) ; 0 ; 0];
Barc_21=[-B(2,1)/C_values(2) ; 0 ; 0];
Barc_31=[-B(3,1)/C_values(3) ; 0 ; 0];
Barc_41=[-B(4,1)/C_values(4) ; 0 ; 0];

Barc_12=[-B(1,2)/C_values(1) ; 0 ; 0];
Barc_22=[-B(2,2)/C_values(2) ; 0 ; 0];
Barc_32=[-B(3,2)/C_values(3) ; 0 ; 0];
Barc_42=[-B(4,2)/C_values(4) ; 0 ; 0];

Barc_13=[-B(1,3)/C_values(1) ; 0 ; 0];
Barc_23=[-B(2,3)/C_values(2) ; 0 ; 0];
Barc_33=[-B(3,3)/C_values(3) ; 0 ; 0];
Barc_43=[-B(4,3)/C_values(4) ; 0 ; 0];

Barc_14=[-B(1,4)/C_values(1) ; 0 ; 0];
Barc_24=[-B(2,4)/C_values(2) ; 0 ; 0];
Barc_34=[-B(3,4)/C_values(3) ; 0 ; 0];
Barc_44=[-B(4,4)/C_values(4) ; 0 ; 0];

BarC=[Barc_11, Barc_12, Barc_13, Barc_14;
    Barc_21, Barc_22, Barc_23, Barc_24;
    Barc_31, Barc_32, Barc_33, Barc_34;
    Barc_41, Barc_42, Barc_43, Barc_44];



D = blkdiag(0, 0, 1);
D=kron(eye(4), D)
% % Load local controllers and resulting passivity indices
% for i = 1:N
%     p_i = pVals(i);
%     obj.vehicles(i + 1).synthesizeLocalControllersParameterized(2, p_i);
% end

% Creating the adjacency matrix, null matrix, and cost matrix
A = zeros(N, N);

adjMatBlock = cell(N, N);
nullMatBlock = cell(N, N);
costMatBlock = cell(N, N);

for i = 1:N
    for j = 1:N
        % Structure of K_ij (which is a 3x3 matrix) should be embedded here
        if i ~= j
            if A(j, i) == 1
                adjMatBlock{i, j} = [0, 0, 0; 1, 1, 1; 0, 0, 0];
                nullMatBlock{i, j} = [1, 1, 1; 0, 0, 0; 1, 1, 1];
                costMatBlock{i, j} = 1 * [0, 0, 0; 1, 1, 1; 0, 0, 0];
            else
                adjMatBlock{i, j} = [0, 0, 0; 0, 0, 0; 0, 0, 0];
                nullMatBlock{i, j} = [1, 1, 1; 0, 0, 0; 1, 1, 1];
                costMatBlock{i, j} = (20 / N) * abs(i - j) * [0, 0, 0; 1, 1, 1; 0, 0, 0];
            end
        else
            adjMatBlock{i, j} = [0, 0, 0; 1, 1, 1; 0, 0, 0];
            nullMatBlock{i, j} = [1, 1, 1; 0, 0, 0; 1, 1, 1];
            costMatBlock{i, j} = 0 * [0, 0, 0; 1, 1, 1; 0, 0, 0];
        end
    end 
end

adjMatBlock = cell2mat(adjMatBlock);
nullMatBlock = cell2mat(nullMatBlock);
costMatBlock = cell2mat(costMatBlock);

% Set up the LMI problem
solverOptions = sdpsettings('solver', 'mosek', 'verbose', 0);
I = eye(3 * N);
I_n = eye(3);
I_bar = eye(1);
O = zeros(3 * N);

% Whether to use a soft or hard graph constraint
isSoft = 1;

Q = sdpvar(3*N, 3*N, 'full'); 
P = sdpvar(N, N, 'diagonal');
BarP = sdpvar(4, 4, 'full');
gammaSq = sdpvar(1, 1, 'full');

X_11=[];
X_p_11 = [];
BarX_Barp_11 = [];
X_p_12 = [];
BarX_p_12 = [];
X_12 = [];
BarX_12 = [];
X_p_22 = [];
BarX_Barp_22 = [];

for i = 1:N
    nu_i = nuValues{i};
    rho_i = rhoValues{i};
    nuBar_i = nuValues_Bar{i};
    rhoBar_i = rhoValues_Bar{i};

    X_11= blkdiag(X_11, -nu_i * I_n);
    X_p_11 = blkdiag(X_p_11, -nu_i * P(i, i) * I_n);
    BarX_Barp_11 = blkdiag(BarX_Barp_11, -nuBar_i * BarP(i, i) * I_bar);
    X_p_12 = blkdiag(X_p_12, 0.5 * P(i, i) * I_n);
    BarX_p_12 = blkdiag(BarX_p_12, 0.5 * BarP(i, i) * I_bar);
    X_12 = blkdiag(X_12, (-1 / (2 * nu_i)) * I_n);
    BarX_12 = blkdiag(BarX_12, (-1 / (2 * nuBar_i)) * I_bar);
    X_p_22 = blkdiag(X_p_22, -rho_i * P(i, i) * I_n);
    BarX_Barp_22 = blkdiag(BarX_Barp_22, -rhoBar_i * BarP(i, i) * I_bar);
end

X_p_21 = X_p_12';
BarX_p_21 = BarX_p_12';
X_21 = X_12';
BarX_21 = BarX_12';

% Objective Function
costFun0 = sum(sum(Q .* costMatBlock));

% Minimum Budget Constraints
con0 = costFun0 >= 0.0001;

% Basic Constraints
con1 = P >= 0;

% Constraints related to the LMI problem
con2 = [X_p_11, O, O, Q, X_p_11 * BarC, X_p_11;
        O, BarX_Barp_11, O, BarX_Barp_11 * C, O, O;
        O, O, I, D, O, O;
        Q', C' * BarX_Barp_11, D', -Q' * X_12 - X_21 * Q - X_p_22, -X_21 * X_11 * BarC - X_11 * C * BarX_12, -X_21 * X_p_11;
        BarC' * X_p_11, O, O, -X_p_11 * BarC * X_12 - BarX_21 * BarX_Barp_11 * C, -BarX_Barp_22, O;
        X_p_11, O, O, -X_p_11 * X_12, O, gammaSq * I] >= 0;

% Structural constraints
% con3 = Q .* (nullMatBlock == 1) == O;  % Structural limitations (due to the format of the control law)
% con4 = Q .* (adjMatBlock == 0) == O;   % Graph structure: hard constraint

constraints = [con0, con1, con2];
% Total Cost and Constraints
% if isSoft
%     % Soft constraints (without the hard graph constraint con7)
%     constraints = [con0, con1, con2, con3];
% else
%     % Hard constraints (with the hard graph constraint con7)
%     constraints = [con0, con1, con2, con3, con4];
% end

% Define the objective function
costFunction = 1 * costFun0 + 1 * gammaSq;

% Solve the optimization problem
solution = optimize(constraints, [costFunction], solverOptions);
status = solution.problem == 0;  % Check if the optimization problem is successfully solved

% Retrieve values
costFun0Val = value(costFun0);
costFunVal = value(costFunction);
PVal = value(P);
QVal = value(Q);

% Additional information (commented out for clarity)
% con2Val = value([DMat, MMat; MMat', ThetaMat]);
% eigVals = eig(con2Val);
% minEigVal = min(eigVals);
% con2Val = value(con2);

% tempMat = value([DMat, MMat; MMat', ThetaMat]);
% detTempMat = det(tempMat);

X_p_11Val = value(X_p_11);
X_p_21Val = value(X_p_21);

gammaSqVal = value(gammaSq);

% Calculate K_ij blocks
M_neVal = X_p_11Val \ QVal;
M_neVal(nullMatBlock == 1) = 0;

maxNorm = 0;
K = cell(N, N);

for i = 1:N
    for j = 1:N
        % (i,j)-th (3 x 3) block
        K{i, j} = M_neVal(3 * (i - 1) + 1:3 * i, 3 * (j - 1) + 1:3 * j);
        normVal = max(max(abs(K{i, j})));

        % Update maxNorm if necessary
        if normVal > maxNorm
            maxNorm = normVal;
        end
    end
end

% Filter out extremely small interconnections
for i = 1:N
    for j = 1:N
        if i ~= j
            if isSoft
                K{i, j}(abs(K{i, j}) < 0.0001 * maxNorm) = 0;
            else
                if A(i + 1, j + 1) == 0
                    K{i, j} = zeros(3);
                end
            end
        end

        % Apply additional filtering based on magnitude
        K_ijMax = max(abs(K{i, j}(:)));
        K{i, j}(abs(K{i, j}) < 0.01 * K_ijMax) = 0;
    end
end

% Display and update topology and controller gains
K
% obj.loadTopologyFromK2(K);
% obj.loadControllerGains2(K);

function obj = DG(numDGs)

            obj.platoonIndex = numDGs;
                      
           
            obj.topology = Topology(n_k); % Generate a topology
            obj.updateNeighbors();   % update the neighbor information of each vehicle object inside obj.vehicles based on obj.topology
            obj.loadDefaultControllerGains(); % based on the neighbor connections, load some controller gains

        end

