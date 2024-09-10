% This function constructs the control matrices C, BarC, and H, and 
% formulates the optimization problem with appropriate constraints and 
% objective functions to ensure the stability and performance of the 
% microgrid under a given communication topology defined by the adjacency 
% matrix A_ij. The function also computes the optimal gain matrices K for 
% each pair of DGs based on the optimization results.

function [DG,Line,statusGlobalController,gammaTildeVal,K,C,BarC,H,P_iVal,P_lVal] = globalControlDesign(DG,Line,A_ij,B_il,BarGamma,isSoft)

numOfDGs = size(B_il,1);
numOfLines = size(B_il,2);

%% Creating C , BarC , and H Matrices

% Create C Matrix
C = zeros(numOfLines, numOfDGs * 3);

for l = 1:numOfLines
    for i = 1:numOfDGs
        C(l, (i-1) * 3 + 1) = B_il(i, l);
    end
end



% Create BarC Matrix
BarC = zeros(numOfDGs * 3, numOfLines);

for i = 1:numOfDGs
    for l = 1:numOfLines
        Ct = DG{i}.C;
        BarC((i-1)*3 + 1, l) = -B_il(i, l)/Ct;
    end
end


% Create H Matrix

H_i = [0, 0 , 1];
H = [];
for i = 1:numOfDGs
    H = blkdiag(H, H_i);
end




%% Creating the adjacency matrix, null matrix, and cost matrix

A = A_ij; % Adjacency matrix of the DG-DG communication topology

adjMatBlock = cell(numOfDGs, numOfDGs);
nullMatBlock = cell(numOfDGs, numOfDGs);
costMatBlock = cell(numOfDGs, numOfDGs);

for i = 1:numOfDGs
    for j = 1:numOfDGs
        % Structure of K_ij (which is a 3x3 matrix) should be embedded here
        if i ~= j
            if A(j, i) == 1
                adjMatBlock{i, j} = [0, 0, 0; 1, 1, 1; 0, 0, 0];
                nullMatBlock{i, j} = [1, 1, 1; 0, 0, 0; 1, 1, 1];
                costMatBlock{i, j} = 0.01 * [0, 0, 0; 1, 1, 1; 0, 0, 0]; %%% Play with this
            else
                adjMatBlock{i, j} = [0, 0, 0; 0, 0, 0; 0, 0, 0];
                nullMatBlock{i, j} = [1, 1, 1; 0, 0, 0; 1, 1, 1];
                dist_ij = norm(DG{i}.coordinates - DG{j}.coordinates);  % In platoons: (20 / numOfDGs) * abs(i - j) 
                costMatBlock{i, j} = dist_ij * [0, 0, 0; 1, 1, 1; 0, 0, 0]; %%% Play with this
            end
        else
            adjMatBlock{i, j} = [0, 0, 0; 1, 1, 1; 0, 0, 0];
            nullMatBlock{i, j} = [1, 1, 1; 0, 0, 0; 1, 1, 1];
            costMatBlock{i, j} = 0 * [0, 0, 0; 1, 1, 1; 0, 0, 0]; %%% Play with this
        end
    end 
end

%%%% Comment: Use the following variable.
adjMatBlock = cell2mat(adjMatBlock);  %%% This is only for enforcing hard garaph constraints
nullMatBlock = cell2mat(nullMatBlock); %%% This is needed in any case as it is a structural constraint
costMatBlock = cell2mat(costMatBlock);   %%% Note that this matrix contains the c_ij information required for the objective function (46a)


%% Variables corresponding to DGs like 

I = eye(3*numOfDGs);
I_N = eye(numOfDGs);
gammaTilde = sdpvar(1, 1,'full');
GammaTilde = gammaTilde*I;

Q = sdpvar(3*numOfDGs, 3*numOfDGs, 'full'); 

P_i = sdpvar(numOfDGs, numOfDGs, 'diagonal');
% for i = 1:numOfDGs
%     P_i(i,i) = sdpvar(1, 1, 'full');
% end


P_l = sdpvar(numOfLines, numOfLines, 'diagonal');
% for l = 1:numOfLines
%     P_l(l,l) = sdpvar(1, 1,'full');
% end

% Fixed Values
% baseP_i = 1e-4;                              % Define the fixed value
% P_i = zeros(numOfDGs); 
% 
% for i = 1:numOfDGs
%     variation = baseP_i * (2 * rand() - 1) * 0.1; % Small variation
%     P_i(i,i) = baseP_i + variation;                 % Set the value in the diagonal
% end
% 
% 
% baseP_l = 1e-4;                             % Define the base value for P_l
% P_l = zeros(numOfLines); 
% 
% for l = 1:numOfLines
%     variation = baseP_l * (2 * rand() - 1) * 0.1; % Small variation
%     P_l(l,l) = baseP_l + variation;                 % Set the value in the diagonal
% end





I_n = eye(3);

X_p_11 = [];
X_p_12 = [];
X_12 = [];
X_p_22 = [];
for i = 1:1:numOfDGs
          
    nu_i = DG{i}.nu;
    rho_i = DG{i}.rho;

    % For DGs, according to equation (33), numOfInouts = numOfOutputs = 3
    X_i_11 = -nu_i*I_n;    %inputs x inputs
    X_i_12 = 0.5*I_n;      %inputs x outputs
    X_i_22 = -rho_i*I_n;   %outputs x outputs

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

for l = 1:1:numOfLines
    
    nu_l = Line{l}.nu;
    rho_l = Line{l}.rho;

    % For Lines, according to equation (31), numOfInputs = numOfOutputs = 1
    BarX_l_11 = -nu_l*I_bar;    %inputs x inputs
    BarX_l_12 = 0.5*I_bar;      %inputs x outputs
    BarX_l_22 = -rho_l*I_bar;   %outputs x outputs

    BarX_Barp_11 = blkdiag(BarX_Barp_11, P_l(l,l)*BarX_l_11);
    BarX_Barp_12 = blkdiag(BarX_Barp_12, P_l(l,l)*BarX_l_12);
    BarX_Barp_22 = blkdiag(BarX_Barp_22, P_l(l,l)*BarX_l_22);
    
    BarX_12 = blkdiag(BarX_12, (-1 / (2 * nu_l)) * I_bar);
    
end
BarX_Barp_21 = BarX_Barp_12';
BarX_21 = BarX_12';



%% Constraints 
constraints = [];

% Constraints in (46b)
for i = 1:numOfDGs
    con1 = P_i(i,i) >= 0;
    constraints = [constraints, con1];
end

% Constraints in (46c)
for l = 1:numOfLines
    con2 = P_l(l,l) >= 0;
    constraints = [constraints, con2];
end

% Constraints in (46d)
con3_1 = gammaTilde >= 0;
con3_2 = gammaTilde <= BarGamma;
constraints = [constraints, con3_1, con3_2];

% Constraint in (47)
O_n = zeros(numOfDGs, 3*numOfDGs);
O_bar = zeros(numOfLines);
O_N = zeros(3*numOfDGs, numOfLines);
O = zeros(numOfDGs, numOfLines);
O_3N = zeros(3*numOfDGs);

Mat1 = [X_p_11];

Mat2 = [X_p_11, O_N;
         O_N', BarX_Barp_11];

Mat3 = [X_p_11, O_N, O_n';
         O_N', BarX_Barp_11, O';
         O_n, O, I_N];

Mat4 = [X_p_11, O_N, O_n', Q;
        O_N', BarX_Barp_11, O', BarX_Barp_11 * C;
        O_n, O, I_N, H;
        Q', C' * BarX_Barp_11', H', -Q' * X_12 - X_21 * Q - X_p_22];

Mat5 = [X_p_11, O_N, O_n', Q, X_p_11 * BarC;
        O_N', BarX_Barp_11, O', BarX_Barp_11 * C, O_bar;
        O_n, O, I_N, H, O;
        Q', C' * BarX_Barp_11', H', -Q' * X_12 - X_21 * Q - X_p_22, -X_21 * X_p_11 * BarC - C' * BarX_Barp_11' * BarX_12;
        BarC' * X_p_11', O_bar, O', -BarC' * X_p_11' * X_12 - BarX_21 * BarX_Barp_11 * C, -BarX_Barp_22];


Mat6 = [X_p_11, O_N, O_n', Q, X_p_11 * BarC, X_p_11;
        O_N', BarX_Barp_11, O', BarX_Barp_11 * C, O_bar, O_N';
        O_n, O, I_N, H, O, O_n;
        Q', C' * BarX_Barp_11', H', -Q' * X_12 - X_21 * Q - X_p_22, -X_21 * X_p_11 * BarC - C' * BarX_Barp_11' * BarX_12, -X_21 * X_p_11;
        BarC' * X_p_11', O_bar, O', -BarC' * X_p_11' * X_12 - BarX_21 * BarX_Barp_11 * C, -BarX_Barp_22, O_N';
        X_p_11, O_N, O_n', -X_p_11' * X_12, O_N, GammaTilde];

Mat4_1 = [X_p_11, X_p_11 * BarC; 
        BarC' * X_p_11', -BarX_Barp_22];
% 
% 
% % Mat4_1 = [-BarX_Barp_22];
con4_1 = Mat4_1 >= 0;


T = Mat4;            
con4 = T >= 0;
% constraints = [constraints, con4];
constraints = [constraints, con4, con4_1];

% Structural constraints
con5 = Q.*(nullMatBlock==1) == O_3N;     % Structural limitations (due to the format of the control law)
constraints = [constraints, con5];

% Objective Function
normType = 2;
costFun0 = 1*norm(Q.*costMatBlock,normType);
% costFun0 = sum(sum(Q.*costMatBlock)); %%% Play with this

% Minimum Budget Constraints
con6 = costFun0 >= 0.001;  %%% Play with this
% constraints = [constraints, con6];

% Hard Graph Constraints (forcing K_ij = K_ji = 0 if i and j are not communication neighbors)
con7 = Q.*(adjMatBlock==0) == O_3N;      % Graph structure : hard constraint

if isSoft
    % Try to get a communication topology that is as much similar/colse as possible to the given communication tpology (by A_ij adjacency matrix)
    constraints = constraints; % Without the hard graph constraint con7
    costFun = 1*costFun0 + 1*gammaTilde; % soft %%% Play with this
else 
    % Follow strictly the given communication tpology (by A_ij adjacency matrix)
    constraints = [constraints, con7]; % With the hard graph constraint con7
    costFun = 1*costFun0 + 1*gammaTilde; % hard (same as soft) %%% Play with this
end

%% Solve the LMI problem (47)

solverOptions = sdpsettings('solver', 'mosek', 'verbose', 1, 'debug', 1);

sol = optimize(constraints,costFun,solverOptions);

statusGlobalController = sol.problem == 0;   

%% Extract variable values

% Study the components of T (all the 1x1 and 2x2 blocks we considered in
% Theorem 2 to find necessary conditions)
TVal = value(T);
TValEigs = eig(TVal);

T1Val = value(X_p_11);
T1ValEigs = eig(T1Val);

T2Val = value(BarX_Barp_11);
T2ValEigs = eig(T2Val);

T3Val = value(-Q' * X_12 - X_21 * Q - X_p_22);
T3ValEigs = eig(T3Val);

T4Val = value(- X_p_22);
T4ValEigs = eig(T4Val);

T5Val = value(-Q' * X_12 - X_21 * Q);
T5ValEigs = eig(T5Val);
% 
% T4_1Val = value(Mat4_1);
% T4_1ValEigs = eig(T4_1Val);

P_iVal = diag(value(P_i));
P_lVal = diag(value(P_l));







gammaTildeVal = value(gammaTilde);
QVal = value(Q);
X_p_11Val = value(X_p_11);
KVal = X_p_11Val \ QVal;
costFun0Val = value(costFun0);
costFunVal = value(costFun);



% Load KVal elements into a cell structure K{i,j} (i.e., partitioning KVal
% into N\times N blocks)
% Obtaining K_ij blocks
KVal(nullMatBlock == 1) = 0;
maxNorm = 0;
for i = 1:1:numOfDGs
    for j = 1:1:numOfDGs
        K{i,j} = KVal(3*(i-1)+1:3*i , 3*(j-1)+1:3*j); % (i,j)-th (3 x 3) block
        normVal = max(max(abs(K{i,j})));
        if normVal>maxNorm 
            maxNorm = normVal;
        end
    end
end

% Filter the K_ij values (weed out the ones with the smallest magnitudes)
% filtering out extremely small interconnections
for i=1:1:numOfDGs
    for j=1:1:numOfDGs
        if i~=j
            if isSoft
                K{i,j}(abs(K{i,j})<0.0001*maxNorm) = 0;                       
            else
                if A(j+1,i+1)==0
                    K{i,j} = zeros(3);
                end
            end
        end

        K_ijMax = max(abs(K{i,j}(:)));
        K{i,j}(abs(K{i,j})<0.01*K_ijMax) = 0;

    end
end

% Load the K_ij values to the DGs
for i = 1:1:numOfDGs
    for j = 1:1:numOfDGs
        DG{i}.K{j} = K{i,j};  
        % K = K{i,j};
    end
end

% Load the K_ij values to the DGs
for i = 1:numOfDGs
    DG{i}.K = cell( 1,numOfDGs); % Initialize K as a cell array
    for j = 1:numOfDGs
        DG{i}.K{j} = K(i,j); % Use parentheses for numeric array indexing
    end
end

end