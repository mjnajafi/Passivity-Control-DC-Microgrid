function [DG, Line, statusGlobalController, gammaTildeVal, K, C, BarC, H, Bar_H, P_iVal, P_lVal] = globalControlDesign(DG, Line, A_ij, B_il, BarGamma, isSoft)

numOfDGs = size(B_il, 1);
numOfLines = size(B_il, 2);
epsilon = 0.000001; % Minimum value
debugMode = 0; % Set to 1 for detailed debugging output

%% Creating C, BarC, and H Matrices

% Create C Matrix (L x 3N)
C = zeros(numOfLines, numOfDGs * 3);
for l = 1:numOfLines
    for i = 1:numOfDGs
        C(l, (i-1) * 3 + 1) = B_il(i, l);
    end
end

% Create BarC Matrix (3N x L)
BarC = zeros(numOfDGs * 3, numOfLines);
for i = 1:numOfDGs
    for l = 1:numOfLines
        Ct = DG{i}.C;
        BarC((i-1)*3 + 1, l) = -B_il(i, l)/Ct;
    end
end

% Define dimensions for state spaces
totalDGStates = 3 * numOfDGs;
totalLineStates = numOfLines;
totalOutputDim = totalDGStates + totalLineStates; % This is 3N+L

% Create H matrix (output mapping for DGs) - 3N x 3N
H = eye(totalDGStates);  % Identity matrix for DG states (eye(3) for each DG)

% Create Bar_H matrix (output mapping for Lines) - L x L
Bar_H = eye(totalLineStates);  % Identity matrix for line states

% Create H_c according to equation (70): H_c = [H; 0] - (3N+L) x 3N
H_c = [H; zeros(totalLineStates, totalDGStates)];

% Create Bar_H_c according to equation (70): Bar_H_c = [0; Bar_H] - (3N+L) x L
Bar_H_c = [zeros(totalDGStates, totalLineStates); Bar_H];

if debugMode
    fprintf('H dimensions: [%d x %d]\n', size(H, 1), size(H, 2));
    fprintf('Bar_H dimensions: [%d x %d]\n', size(Bar_H, 1), size(Bar_H, 2));
    fprintf('H_c dimensions: [%d x %d]\n', size(H_c, 1), size(H_c, 2));
    fprintf('Bar_H_c dimensions: [%d x %d]\n', size(Bar_H_c, 1), size(Bar_H_c, 2));
    fprintf('Total output dimension: %d\n', totalOutputDim);
end

%% Creating the adjacency matrix, null matrix, and cost matrix
A = A_ij; % Adjacency matrix of the DG-DG communication topology


adjMatBlock = cell(numOfDGs, numOfDGs);
nullMatBlock = cell(numOfDGs, numOfDGs);
costMatBlock = cell(numOfDGs, numOfDGs);


for i = 1:numOfDGs
    for j = 1:numOfDGs
        if i ~= j  % Only for non-diagonal terms
            if A(j, i) == 1
                % Structure for direct difference gains
                adjMatBlock{i, j} = [0, 0, 0; 0, 1, 0; 0, 0, 0];
                nullMatBlock{i, j} = [1, 1, 1; 1, 0, 1; 1, 1, 1];
                % Cost based on physical distance
                dist_ij = norm(DG{i}.coordinates - DG{j}.coordinates);
                costMatBlock{i, j} = 0.5*dist_ij^2 * [0, 0, 0; 0, 1, 0; 0, 0, 0];
                % costMatBlock{i, j} = zeros(3);
                % costMatBlock{i, j} = (1 + 0.5*randn()) * [0, 0, 0; 0, 1, 0; 0, 0, 0];
                % costMatBlock{i, j} = 1 * rand() * [0, 0, 0; 0, 1, 0; 0, 0, 0];
                % costMatBlock{i, j} = (dist_ij^2 + 5*abs(i-j)) * [0, 0, 0; 0, 1, 0; 0, 0, 0];

            else
                % Non-connected DGs
                adjMatBlock{i, j} = zeros(3);
                nullMatBlock{i, j} = [1, 1, 1; 1, 0, 1; 1, 1, 1];
                dist_ij = norm(DG{i}.coordinates - DG{j}.coordinates);
                costMatBlock{i, j} = dist_ij^2 * [0, 0, 0; 0, 1, 0; 0, 0, 0];
            end
        else
            % Diagonal terms (i == j) - all zero as we don't need self-gains
            adjMatBlock{i, j} = zeros(3);
            nullMatBlock{i, j} = ones(3);  % Force diagonal blocks to be zero
            costMatBlock{i, j} = zeros(3);
        end
    end
end

adjMatBlock = cell2mat(adjMatBlock);
nullMatBlock = cell2mat(nullMatBlock);
costMatBlock = cell2mat(costMatBlock);

%% Variables corresponding to DGs and lines
gammaTilde = sdpvar(1, 1,'full');

Q = sdpvar(3*numOfDGs, 3*numOfDGs, 'full'); 

P_i = sdpvar(numOfDGs, numOfDGs, 'diagonal');
P_l = sdpvar(numOfLines, numOfLines, 'diagonal');

I_n = eye(3);

X_p_11 = [];
X_p_12 = [];
X_12 = [];
X_p_22 = [];
for i = 1:numOfDGs     
    nu_i = DG{i}.nu;
    rho_i = DG{i}.rho;

    % For DGs, according to equation (73)
    X_i_11 = -nu_i*I_n;    % inputs x inputs
    X_i_12 = 0.5*I_n;      % inputs x outputs
    X_i_22 = -rho_i*I_n;   % outputs x outputs

    X_p_11 = blkdiag(X_p_11, P_i(i,i)*X_i_11);
    X_p_12 = blkdiag(X_p_12, P_i(i,i)*X_i_12);
    X_p_22 = blkdiag(X_p_22, P_i(i,i)*X_i_22);
    X_12 = blkdiag(X_12, (-1 / (2 * nu_i)) * I_n);
end
X_p_21 = X_p_12';
X_21 = X_12';

I_bar = eye(1);

BarX_Barp_11 = [];
BarX_Barp_12 = [];
BarX_12 = [];
BarX_Barp_22 = [];

for l = 1:numOfLines
    nu_l = Line{l}.nu;
    rho_l = Line{l}.rho;

    % For Lines, according to equation (74)
    BarX_l_11 = -nu_l*I_bar;    % inputs x inputs
    BarX_l_12 = 0.5*I_bar;      % inputs x outputs
    BarX_l_22 = -rho_l*I_bar;   % outputs x outputs

    BarX_Barp_11 = blkdiag(BarX_Barp_11, P_l(l,l)*BarX_l_11);
    BarX_Barp_12 = blkdiag(BarX_Barp_12, P_l(l,l)*BarX_l_12);
    BarX_Barp_22 = blkdiag(BarX_Barp_22, P_l(l,l)*BarX_l_22);
    
    BarX_12 = blkdiag(BarX_12, (-1 / (2 * nu_l)) * I_bar);
end
BarX_Barp_21 = BarX_Barp_12';
BarX_21 = BarX_12';

%% Define disturbance matrices E and Ebar for DGs and Lines
% Build individual disturbance matrices for DGs
E = [];
for i = 1:numOfDGs
    C_ti = DG{i}.C;
    L_ti = DG{i}.L;
    E_i = diag([1/C_ti, 1/L_ti, 1]);
    
    % Store E_i in the DG structure
    DG{i}.E = E_i;
    
    E = blkdiag(E, E_i);
end

% Build individual disturbance matrices for Lines
Ebar = [];
for l = 1:numOfLines
    L_l = Line{l}.L;
    Ebar_l = 1/L_l;
    
    % Store Ebar_l in the Line structure
    Line{l}.Ebar = Ebar_l;
    
    Ebar = blkdiag(Ebar, Ebar_l);
end

% Define E_c and Ebar_c as in equation (69)
E_c = [E, zeros(size(E,1), size(Ebar,2))];
Ebar_c = [zeros(size(Ebar,1), size(E,2)), Ebar];

%% Construct the LMI matrix W following equation (80)

% Block (1,1): X_p_11 (3N x 3N)
W11 = X_p_11;

% Block (1,2): 0 (3N x L)
W12 = zeros(totalDGStates, totalLineStates);

% Block (2,1): 0 (L x 3N)
W21 = zeros(totalLineStates, totalDGStates);

% Block (2,2): BarX_Barp_11 (L x L)
W22 = BarX_Barp_11;

% Block (1,3): 0 (3N x (3N+L))
W13 = zeros(totalDGStates, totalOutputDim);

% Block (2,3): 0 (L x (3N+L))
W23 = zeros(totalLineStates, totalOutputDim);

% Block (3,1): 0 ((3N+L) x 3N)
W31 = zeros(totalOutputDim, totalDGStates);

% Block (3,2): 0 ((3N+L) x L)
W32 = zeros(totalOutputDim, totalLineStates);

% Block (3,3): I ((3N+L) x (3N+L))
W33 = eye(totalOutputDim);

% Block (1,4): Q (3N x 3N)
W14 = Q;

% Block (2,4): BarX_Barp_11 * C (L x 3N)
W24 = BarX_Barp_11 * C;

% Block (3,4): H_c ((3N+L) x 3N) 
W34 = H_c;

% Block (4,1): Q' (3N x 3N)
W41 = Q';

% Block (4,2): C' * BarX_Barp_11' (3N x L)
W42 = C' * BarX_Barp_11';

% Block (4,3): H_c' (3N x (3N+L))
W43 = H_c';

% Block (4,4): -Q' * X_12 - X_21 * Q - X_p_22 (3N x 3N)
W44 = -Q' * X_12 - X_21 * Q - X_p_22;

% Block (1,5): X_p_11 * BarC (3N x L)
W15 = X_p_11 * BarC;

% Block (2,5): 0 (L x L)
W25 = zeros(totalLineStates, totalLineStates);

% Block (3,5): Bar_H_c ((3N+L) x L) 
W35 = Bar_H_c;

% Block (4,5): -X_21 * X_p_11 * BarC - C' * BarX_Barp_11' * BarX_12 (3N x L)
W45 = -X_21 * X_p_11 * BarC - C' * BarX_Barp_11' * BarX_12;

% Block (5,1): (X_p_11 * BarC)' (L x 3N)
W51 = (X_p_11 * BarC)';

% Block (5,2): 0 (L x L)
W52 = zeros(totalLineStates, totalLineStates);

% Block (5,3): Bar_H_c' (L x (3N+L))
W53 = Bar_H_c';

% Block (5,4): (-X_21 * X_p_11 * BarC - C' * BarX_Barp_11' * BarX_12)' (L x 3N)
W54 = W45';

% Block (5,5): -BarX_Barp_22 (L x L)
W55 = -BarX_Barp_22;

% Create the disturbance-related gamma block
distSize = size(E_c, 2);
Gamma_tilde = gammaTilde * eye(distSize);

% Block (1,6): X_p_11 * E_c (3N x DistSize)
W16 = X_p_11 * E_c;

% Block (2,6): BarX_Barp_11 * Ebar_c (L x DistSize)
W26 = BarX_Barp_11 * Ebar_c;

% Block (3,6): 0 ((3N+L) x DistSize)
W36 = zeros(totalOutputDim, distSize);

% Block (4,6): -X_21 * X_p_11 * E_c (3N x DistSize)
W46 = -X_21 * X_p_11 * E_c;

% Block (5,6): -BarX_21 * BarX_Barp_11 * Ebar_c (L x DistSize)
W56 = -BarX_21 * BarX_Barp_11 * Ebar_c;

% Block (6,1): (X_p_11 * E_c)' (DistSize x 3N)
W61 = W16';

% Block (6,2): (BarX_Barp_11 * Ebar_c)' (DistSize x L)
W62 = W26';

% Block (6,3): 0 (DistSize x (3N+L))
W63 = zeros(distSize, totalOutputDim);

% Block (6,4): (-X_21 * X_p_11 * E_c)' (DistSize x 3N)
W64 = W46';

% Block (6,5): (-BarX_21 * BarX_Barp_11 * Ebar_c)' (DistSize x L)
W65 = W56';

% Block (6,6): Gamma_tilde (DistSize x DistSize)
W66 = Gamma_tilde;

% Assemble the blocks into W
W = [
    W11, W12, W13, W14, W15, W16;
    W21, W22, W23, W24, W25, W26;
    W31, W32, W33, W34, W35, W36;
    W41, W42, W43, W44, W45, W46;
    W51, W52, W53, W54, W55, W56;
    W61, W62, W63, W64, W65, W66
];

if debugMode
    % Print dimensions of each block
    fprintf('\n---- Block Dimensions Check ----\n');
    fprintf('W11 (X_p_11): [%d x %d] - Should be [%d x %d]\n', size(W11), totalDGStates, totalDGStates);
    fprintf('W12 (0): [%d x %d] - Should be [%d x %d]\n', size(W12), totalDGStates, totalLineStates);
    fprintf('W13 (0): [%d x %d] - Should be [%d x %d]\n', size(W13), totalDGStates, totalOutputDim);
    fprintf('W14 (Q): [%d x %d] - Should be [%d x %d]\n', size(W14), totalDGStates, totalDGStates);
    fprintf('W15 (X_p_11*BarC): [%d x %d] - Should be [%d x %d]\n', size(W15), totalDGStates, totalLineStates);
    
    fprintf('W31 (0): [%d x %d] - Should be [%d x %d]\n', size(W31), totalOutputDim, totalDGStates);
    fprintf('W33 (I): [%d x %d] - Should be [%d x %d]\n', size(W33), totalOutputDim, totalOutputDim);
    fprintf('W34 (H_c): [%d x %d] - Should be [%d x %d]\n', size(W34), totalOutputDim, totalDGStates);
    fprintf('W35 (Bar_H_c): [%d x %d] - Should be [%d x %d]\n', size(W35), totalOutputDim, totalLineStates);
    
    fprintf('W44: [%d x %d] - Should be [%d x %d]\n', size(W44), totalDGStates, totalDGStates);
    
    fprintf('Final W: [%d x %d]\n', size(W));
end

%% Constraints 
constraints = [];
constraintTags = {}; % Cell array to hold tags
constraintMats = {}; % Cell array to hold matrices

% constraints = [constraints,
%     norm(Q, 'fro') <= 1e4,
%     norm(P_i, 'fro') <= 1e4,
%     norm(P_l, 'fro') <= 1e4,
%     norm(gammaTilde, 'fro') <= 1e4  % Changed from GammaTilde to gammaTilde
% ];


% Constraints 
for i = 1:numOfDGs
    tagName = ['P_',num2str(i),num2str(i)];
    constraintTags{end+1} = tagName;
    con1 = tag(P_i(i,i) >= epsilon, tagName);
    constraintMats{end+1} = P_i(i,i);

    constraints = [constraints, con1];
end

% Constraints 
for l = 1:numOfLines
    tagName = ['PBar_',num2str(l),num2str(l)];
    constraintTags{end+1} = tagName;
    con2 = tag(P_l(l,l) >= epsilon, tagName);
    constraintMats{end+1} = P_l(l,l);

    constraints = [constraints, con2];
end

% Constraints in (46d)
tagName = ['gammaTilde_low'];
constraintTags{end+1} = tagName;
con3_1 = tag(gammaTilde >= epsilon, tagName);
constraintMats{end+1} = gammaTilde;

tagName = ['gammaTilde_high'];
constraintTags{end+1} = tagName;
con3_2 = tag(gammaTilde <= BarGamma, tagName);
constraintMats{end+1} = gammaTilde;

constraints = [constraints, con3_1, con3_2];

% Main LMI constraint in equation (80)
slack_W = sdpvar(size(W,1), size(W,1), 'symmetric');
tagName = ['W'];
constraintTags{end+1} = tagName;
con4 = tag(W + slack_W >= epsilon, 'W');
constraintMats{end+1} = W;
constraints = [constraints, con4];

% Add auxiliary constraints for numerical stability
% constraints = [constraints,
%     slack_W >= 0,
%     trace(slack_W) <= 1e2
% ];
constraints = [constraints, slack_W >= 0];


% Structural constraints for controller form
tagName = ['Q_Structure'];
constraintTags{end+1} = tagName;
con5 = tag(Q.*(nullMatBlock==1) == zeros(size(Q)), tagName);
constraintMats{end+1} = Q;
constraints = [constraints, con5];

% Hard Graph Constraints if needed
if ~isSoft
    tagName = ['Q_Topology'];
    constraintTags{end+1} = tagName;
    con7 = tag(Q.*(adjMatBlock==0) == zeros(size(Q)), tagName);
    constraintMats{end+1} = Q;
    constraints = [constraints, con7];
end

%% Objective Function
costFun0 = sum(sum(Q.*costMatBlock));

% Minimum Budget Constraints
con6 = costFun0 >= 0.01;
constraints = [constraints, con6];

if isSoft
    alpha = 100;  % When isSoft is 1, emphasize communication cost
else
    alpha = 0;    % When isSoft is 0, exclude communication cost
end

beta = 0.005;       % Robustness - performance cost
slackPenalty = 100;  % Weight for slack terms

% lambda = 0.000001;  % Sparsity promotion factor - adjust as needed
% L1_penalty = 0;
% 
% % Calculate L1 norm of the Q matrix elements where communication is allowed
% for i = 1:numOfDGs
%     for j = 1:numOfDGs
%         if i ~= j  % Skip diagonal elements
%             % Only consider the current-sharing elements (position 2,2 in each block)
%             L1_penalty = L1_penalty + abs(Q(3*(i-1)+2, 3*(j-1)+2));
%         end
%     end
% end


% Total Cost Function
% costFun = alpha * costFun0 + beta * gammaTilde + slackPenalty * norm(slack_W, 'fro');
costFun = alpha * costFun0 + beta * gammaTilde + slackPenalty * trace(slack_W);
% costFun = alpha * costFun0 + beta * gammaTilde + slackPenalty * trace(slack_W) + lambda * L1_penalty;



%% Solve the LMI problem
solverOptions = sdpsettings('solver', 'mosek', 'verbose', 1, 'debug', 0);
sol = optimize(constraints, costFun, solverOptions);

statusGlobalController = sol.problem == 0;   

%% Extract variable values
P_iVal = diag(value(P_i));
P_lVal = diag(value(P_l));

gammaTildeVal = value(gammaTilde);
costFun0Val = value(costFun0);

QVal = value(Q);
X_p_11Val = value(X_p_11);
KVal = X_p_11Val \ QVal;

% Load KVal elements into a cell structure K{i,j}
KVal(nullMatBlock == 1) = 0;
maxNorm = 0;
for i = 1:numOfDGs
    for j = 1:numOfDGs
        K{i,j} = KVal(3*(i-1)+1:3*i, 3*(j-1)+1:3*j);
        normVal = max(max(abs(K{i,j})));
        if normVal > maxNorm 
            maxNorm = normVal;
        end
    end
end

% Filter out small gain values
% for i = 1:numOfDGs
%     for j = 1:numOfDGs
%         if isSoft
%             K{i,j}(abs(K{i,j}) < 0.001 * maxNorm) = 0;
%         else
%             if A(j,i) == 0
%                 K{i,j} = zeros(3);
%             end
%         end
% 
%         K_ijMax = max(abs(K{i,j}(:)));
%         if K_ijMax > 0
%             K{i,j}(abs(K{i,j}) < 0.001 * K_ijMax) = 0;
%         end
%     end
% end

for i=1:1:numOfDGs
    for j=1:1:numOfDGs
        if isSoft
            K{i,j}(abs(K{i,j}) < 0.1 * maxNorm) = 0;
        else
            if A(j,i) == 0
                K{i,j} = zeros(3);
            end
        end

        K_ijMax = max(abs(K{i,j}(:)));
        K{i,j}(abs(K{i,j}) < 0.1 * K_ijMax) = 0;

    end
end

% Load the K_ij values to the DGs
for i = 1:numOfDGs
    DG{i}.K = cell(1,numOfDGs);
    for j = 1:numOfDGs
        DG{i}.K{j} = K{i,j};
    end
end

%% Debugging
if debugMode
    feasibility = check(constraints);
    
    % Combine tags and feasibility into one array for sorting
    combinedList = [feasibility(:), (1:length(feasibility))'];
    
    % Sort based on the first column (feasibility values)
    sortedList = sortrows(combinedList, 1);
    
    % Printing constraint status
    % for i = 1:length(sortedList)
    %     idx = sortedList(i, 2);
    %     if feasibility(idx) < -1e-6
    %         disp(['Constraint "', constraintTags{idx}, '" is violated by ', num2str(feasibility(idx)), ' .']);
    %     else
    %         disp(['Constraint "', constraintTags{idx}, '" is satisfied by ', num2str(feasibility(idx)), ' .']);
    %     end
    % end
    
    % Display eigenvalues of key matrices
    disp('Eigenvalues of W matrix:');
    disp(eig(value(W)));
end
end