function [DG,Line] = globalControlDesign(DG,Line)


%% Creating C , BarC , and H Matrices

% Create C Matrix
C = zeros(numOfDGs, numOfLines * 3);

for l = 1:numOfLines
    for i = 1:numOfDGs
        C(i, (l-1)*3 + 1) = B_il(i, l);
    end
end


% Create BarC Matrix
BarC = zeros(numOfDGs * 3, numOfLines);

for l = 1:numOfLines
    for i = 1:numOfDGs
        
        Ct = DG{i}.C;
        
        BarC((i-1)*3 + 1, l) = -B_il(i, l) / Ct;
    end
end


% Create H Matrix
for i = 1:numOfDGs
    % Define H_i for the current DG
    H_i = [0, 0, 1];
    
    % Determine the column indices for H_i in H
    startCol = (i-1) * 3 + 1;
    endCol = i * 3;
    
    % Fill H with H_i in the correct position
    H(i, startCol:endCol) = H_i;
end
%% Defining some other matrices

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
solverOptions = sdpsettings('solver', 'mosek', 'verbose', 1);
I = eye(3 * N);
I_n = eye(3);
I_bar = eye(1);
O_n = zeros(3 * N);
O_bar=zeros(N);
O = zeros([12 4]);


% Whether to use a soft or hard graph constraint
isSoft = 1;

Q = sdpvar(3*N, 3*N, 'full'); 
P = sdpvar(N, N, 'diagonal');
BarP = sdpvar(4, 4, 'diagonal');
% gammaSq = sdpvar(1, 1, 'full');
gammaSq = 1;

X_p_11 = [];
BarX_Barp_11 = [];
X_p_12 = [];
BarX_p_12 = [];
X_12 = [];
BarX_12 = [];
X_p_22 = [];
BarX_Barp_22 = [];

for i = 1:N
    nu_i = nu{i};
    rho_i = rhoTilde{i};
    nuBar_i = nuBar{i};
    rhoBar_i = rhoBar{i};

    X_p_11 = blkdiag(X_p_11, -nu_i * P(i, i) * I_n);
    BarX_Barp_11 = blkdiag(BarX_Barp_11, -nuBar_i * BarP(i, i) * I_bar);
    X_p_12 = blkdiag(X_p_12, 0.5 * P(i, i) * I_n);
    BarX_p_12 = blkdiag(BarX_p_12, 0.5 * BarP(i, i) * I_bar);
    X_12 = blkdiag(X_12, (-1 / (2 * nu_i)) * I_n);
    BarX_12 = blkdiag(BarX_12, (-0.5 * nuBar_i) * I_bar);
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
con0 = costFun0 >= 0;

% Basic Constraints
con1 = P >= 0;
con2 = BarP >= -1e-6 ;

% Constraints related to the LMI problem
% Mat1 = [X_p_11];

% Mat2 = [X_p_11,O; O', BarX_Barp_11];

% Mat3 = [X_p_11, O, O_n;O', BarX_Barp_11, O'; O_n, O, I];

% Mat4 = [X_p_11, O, O_n, Q;
%           O', BarX_Barp_11, O', BarX_Barp_11 * C;
%           O_n, O, I, D;
%           Q', C' * BarX_Barp_11, D', -Q' * X_12 - X_21 * Q - X_p_22];

% Mat5 = [X_p_11, O, O_n, Q, X_p_11 * BarC; 
%         O', BarX_Barp_11, O', BarX_Barp_11 * C, O_bar;
%         O_n, O, I, D, O; 
%         Q', C' * BarX_Barp_11, D', -Q' * X_12 - X_21 * Q - X_p_22, -X_21 * X_p_11 * BarC - C' * BarX_Barp_11 * BarX_12 ;
%         BarC' * X_p_11, O_bar, O', -BarC' * X_p_11 * X_12 - BarX_21 * BarX_Barp_11 * C, -BarX_Barp_22];

 Mat6 = [X_p_11, O, O_n, Q, X_p_11 * BarC, X_p_11;
        O', BarX_Barp_11, O', BarX_Barp_11 * C, O_bar, O';
        O_n, O, I, D, O, O_n;
        Q', C' * BarX_Barp_11, D', -Q' * X_12 - X_21 * Q - X_p_22, -X_21 * X_p_11 * BarC - C' * BarX_Barp_11 * BarX_12, -X_21 * X_p_11;
        BarC' * X_p_11, O_bar, O', -BarC' * X_p_11 * X_12 - BarX_21 * BarX_Barp_11 * C, -BarX_Barp_22, O';
        X_p_11, O, O_n, -X_p_11 * X_12, O, gammaSq * I];

 
T = [Mat6];

% T = [X_p_11, O, O_n, Q, X_p_11 * BarC, X_p_11;
%         O', BarX_Barp_11, O', BarX_Barp_11 * C, O_bar, O';
%         O_n, O, I, D, O, O_n;
%         Q', C' * BarX_Barp_11, D', -Q' * X_12 - X_21 * Q - X_p_22, -X_21 * X_p_11 * BarC - C' * BarX_Barp_11 * BarX_12, -X_21 * X_p_11;
%         BarC' * X_p_11, O_bar, O', -BarC' * X_p_11 * X_12 - BarX_21 * BarX_Barp_11 * C, -BarX_Barp_22, O';
%         X_p_11, O, O_n, -X_p_11 * X_12, O, gammaSq * I];

con3 = T  >= 0;
% Structural constraints
con4 = Q .* (nullMatBlock == 1) == zeros(12,12);  % Structural limitations (due to the format of the control law)
% con5 = Q .* (adjMatBlock == 0) == zeros(12,12);   % Graph structure: hard constraint

constraints = [con0, con1, con2, con3, con4];
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
z0 = value(con0)
z1 = value(con1)
z2 = value(con2)
z3 = value(con3)
z4 = value(con4)
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









end