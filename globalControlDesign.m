function [DG,Line,statusGlobalController] = globalControlDesign(DG,Line,B_il,numOfDGs,numOfLines)


%% Creating C , BarC , and H Matrices

% Create C Matrix
C = zeros(numOfDGs, numOfLines * 3);

% Fill the C matrix
for l = 1:numOfLines
    for i = 1:numOfDGs
        % Compute the correct column index for C
        columnIndex = (l-1)*3 + 1;
        C(i, columnIndex) = B_il(i, l);
    end
end


% Create BarC Matrix
BarC = zeros(numOfDGs * 3, numOfLines);

for l = 1:numOfLines
    for i = 1:numOfDGs
        Ct = DG{i}.C;
        BarC((i-1)*3 + 1, l) = -B_il(i, l)/Ct;
    end
end


% Create H Matrix
%%%% Comment: Dimentions of H are not correct, check Eq. (40) in the paper
H = zeros(numOfDGs, numOfDGs * 3);

for i = 1:numOfDGs
    H((i-1)*3 + 3, (i-1)*3 + 3) = 1;
end



%% Creating the adjacency matrix, null matrix, and cost matrix

A = zeros(numOfDGs, numOfDGs);

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
                costMatBlock{i, j} = 1 * [0, 0, 0; 1, 1, 1; 0, 0, 0];
            else
                adjMatBlock{i, j} = [0, 0, 0; 0, 0, 0; 0, 0, 0];
                nullMatBlock{i, j} = [1, 1, 1; 0, 0, 0; 1, 1, 1];
                costMatBlock{i, j} = (20 / numOfDGs) * abs(i - j) * [0, 0, 0; 1, 1, 1; 0, 0, 0];
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


%% Variables corresponding to DGs like 
I = eye(3 * numOfDGs);
I_n = eye(3);
I_bar = eye(1);
O_n = zeros(3 * numOfDGs);
O_bar = zeros(numOfDGs);
O = zeros([3*numOfDGs numOfDGs]);

% Initialize cell array
p_i = cell(numOfDGs, 1);
Q_ij = cell(numOfDGs, numOfDGs);

for i = 1:numOfDGs
    for j = 1:numOfDGs
        %%%% Comment: Dimentions of P is not correct, check Eq. (46b) in the
        %%%% paper. Also, use lower-case p_i{i} as they are scalars
        p_i{i} = sdpvar(1, 1, 'full');
        %%%% Comment: Dimentions of Q is not correct, check below Eq. (46) in the
        %%%% paper. Also, use Q_ij{i,j} cell structure
        Q_ij{i,j} = sdpvar(3*numOfDGs, 3*numOfDGs, 'full'); 
        GammaTilde_i{i} = sdpvar(1, 1,'full');
    end
end

for l = 1:1:numOfLines
    %%%% Comment: Dimentions of BarP is not correct, check Eq. (46c) in the
    %%%% paper. Also, use lower-case Barp_l{l} as they are scalars
    Barp_l{l} = sdpvar(1, 1,'full');
end

X_p_11 = [];
BarX_Barp_11 = [];
X_p_12 = [];
BarX_p_12 = [];
X_12 = [];
BarX_12 = [];
X_p_22 = [];
BarX_Barp_22 = [];

for i = 1:1:numOfDGs
          
        nu_i = DG{i}.nu;
        rho_i = DG{i}.rho;
        
        X_p_11 = blkdiag(X_p_11, -nu_i * p_i{i} * I_n);
        X_p_12 = blkdiag(X_p_12, 0.5 * p_i{i} * I_n);
        X_12 = blkdiag(X_12, (-1 / (2 * nu_i)) * I_n);
        X_p_22 = blkdiag(X_p_22, -rho_i * p_i{i} * I_n);
        X_p_21 = X_p_12';
        X_21 = X_12';
end


for l = 1:1:numOfLines
    
    nuBar_l = Line{l}.nu;
    rhoBar_l = Line{l}.rhoBar;

    BarX_Barp_11 = blkdiag(BarX_Barp_11, -nuBar_l * Barp_l{l} * I_bar);
    BarX_p_12 = blkdiag(BarX_p_12, 0.5 * Barp_l{l} * I_bar);
    BarX_12 = blkdiag(BarX_12, (-0.5 * nuBar_l) * I_bar);
    BarX_Barp_22 = blkdiag(BarX_Barp_22, -rhoBar_l * Barp_l{l} * I_bar);
    BarX_p_21 = BarX_p_12';
    BarX_21 = BarX_12';
end

constraints = [];
%% Constraints 
% 
for i = 1:numOfDGs
    for l = 1:numOfLines

        % Objective Function
        costFun0 = sum(sum(Q_ij{i,j} .* costMatBlock));
        
        % Minimum Budget Constraints
        con0 = costFun0 >= 0;
        
        % Basic Constraints
        con1 = p_i{i} >= 0;
        con2 = Barp_l{l} >= 0;
        
        % Constraints related to the LMI problem
        T = [X_p_11, O, O_n, Q_ij{i,j}, X_p_11 * BarC, X_p_11;
                O', BarX_Barp_11, O', BarX_Barp_11 * C, O_bar, O';
                O_n, O, I, H, O, O_n;
                Q_ij{i,j}', C' * BarX_Barp_11, H', -Q_ij{i,j}' * X_12 - X_21 * Q_ij{i,j} - X_p_22, -X_21 * X_p_11 * BarC - C' * BarX_Barp_11 * BarX_12, -X_21 * X_p_11;
                BarC' * X_p_11, O_bar, O', -BarC' * X_p_11 * X_12 - BarX_21 * BarX_Barp_11 * C, -BarX_Barp_22, O';
                X_p_11, O, O_n, -X_p_11 * X_12, O, GammaTilde_i{i} * I];
        
         
        con3 = T  >= 0;
                     
        % Structural constraints
        con4 = Q_ij{i,j} .* (nullMatBlock == 1) == zeros(12,12);  % Structural limitations (due to the format of the control law)
                     
        % Collecting Constraints
        constraints = [con0, con1, con2, con3, con4];

    end
end

%% Solve the LMI problem (47)

% Defining costfunction
costFunction = 1 * costFun0 + 1 * GammaTilde_i{i};

solverOptions = sdpsettings('solver', 'mosek', 'verbose', 1);

sol = optimize(constraints, costFunction, solverOptions);

statusGlobalController = sol.problem == 0;   

%% Extract variable values
for i = 1:1:numOfDGs
    PVal = value(p_i{i});
    QVal = value(Q_ij{i,j});
    X_p_11Val = value(X_p_11);

    % Calculate K_ij blocks
    K = X_p_11Val \ QVal;
    
    % update DG
    DG{i}.PVal = PVal;
    DG{i}.Kij = K;
end












end