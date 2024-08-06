function [DG,Line,statusGlobalController] = globalControlDesign(DG,Line,A_ij,B_il,BarGamma,isSoft)

numOfDGs = size(B_il,1);
numOfLines = size(B_il,2);

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
    H(i, (i-1)*3 + 3) = 1;
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
gammaTilde = sdpvar(1, 1,'full');
GammaTilde = gammaTilde*I;

Q = sdpvar(3*numOfDGs, 3*numOfDGs, 'full'); 

for i = 1:numOfDGs
    p_i{i} = sdpvar(1, 1, 'full');
end

for l = 1:numOfLines
    p_l{l} = sdpvar(1, 1,'full');
end


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

    X_p_11 = blkdiag(X_p_11, p_i{i}*X_i_11);
    X_p_12 = blkdiag(X_p_12, p_i{i}*X_i_12);
    X_p_22 = blkdiag(X_p_22, p_i{i}*X_i_22);
    X_p_21 = X_p_12';
    
    X_12 = blkdiag(X_12, (-1 / (2 * nu_i)) * I_n);
    X_21 = X_12';

end

I_bar = eye(1);

BarX_Barp_11 = [];
BarX_Barp_12 = [];
BarX_12 = [];
BarX_Barp_22 = [];
for l = 1:1:numOfLines
    
    nu_l = Line{l}.nu;
    rho_l = Line{l}.rho;

    % For Lines, according to equation (31), numOfInouts = numOfOutputs = 1
    BarX_l_11 = -nu_l*I_bar;    %inputs x inputs
    BarX_l_12 = 0.5*I_bar;      %inputs x outputs
    BarX_l_22 = -rho_l*I_bar;   %outputs x outputs

    BarX_Barp_11 = blkdiag(BarX_Barp_11, p_l{l}*BarX_l_11);
    BarX_Barp_12 = blkdiag(BarX_Barp_12, p_l{l}*BarX_l_12);
    BarX_Barp_22 = blkdiag(BarX_Barp_22, p_l{l}*BarX_l_22);
    BarX_Barp_21 = BarX_Barp_12';

    BarX_12 = blkdiag(BarX_12, (-1 / (2 * nu_l)) * I_bar);
    BarX_21 = BarX_12';
end


%% Constraints 
constraints = [];

% Constraints in (46b)
for i = 1:numOfDGs
    con1 = p_i{i} >= 0;
    constraints = [constraints, con1];
end

% Constraints in (46c)
for l = 1:numOfLines
    con2 = p_l{l} >= 0;
    constraints = [constraints, con2];
end

% Constraints in (46d)
con3_1 = gammaTilde >= 0;
con3_2 = gammaTilde <= BarGamma;
constraints = [constraints, con3_1, con3_2];

% Constraint in (47)
% O_n = zeros(3*numOfDGs);
% O_bar = zeros(numOfDGs);
O = zeros(3*numOfDGs, 3*numOfDGs]);
%%%% Comment: Fix the following constraint with appropriate "O" variables
T = [X_p_11, O, O_n, Q_ij{i,j}, X_p_11 * BarC, X_p_11;
        O', BarX_Barp_11, O', BarX_Barp_11 * C, O_bar, O';
        O_n, O, I, H, O, O_n;
        Q_ij{i,j}', C' * BarX_Barp_11, H', -Q_ij{i,j}' * X_12 - X_21 * Q_ij{i,j} - X_p_22, -X_21 * X_p_11 * BarC - C' * BarX_Barp_11 * BarX_12, -X_21 * X_p_11;
        BarC' * X_p_11, O_bar, O', -BarC' * X_p_11 * X_12 - BarX_21 * BarX_Barp_11 * C, -BarX_Barp_22, O';
        X_p_11, O, O_n, -X_p_11 * X_12, O, GammaTilde_i{i} * I];
            
con4 = T  >= 0;
constraints = [constraints, con4];

% Structural constraints
con5 = Q.*(nullMatBlock==1)==O;     % Structural limitations (due to the format of the control law)
constraints = [constraints, con5];

% Objective Function
% costFun0 = 1*norm(Q.*costMatBlock,normType);
costFun0 = sum(sum(Q.*costMatBlock)); %%% Play with this

% Minimum Budget Constraints
con6 = costFun0 >= 0.002;  %%% Play with this
constraints = [constraints, con6];

% Hard Graph Constraints (forcing K_ij = K_ji = 0 if i and j are not communication neighbors)
con7 = Q.*(adjMatBlock==0)==O;      % Graph structure : hard constraint

if isSoft
    % Try to get a communication topology that is as much similar/colse as possible to the given communication tpology (by A_ij adjacency matrix)
    cons = constraints; % Without the hard graph constraint con7
    costFun = 1*costFun0 + 1*gammaTilde; % soft %%% Play with this
else 
    % Follow strictly the given communication tpology (by A_ij adjacency matrix)
    cons = [constraints, con7]; % With the hard graph constraint con7
    costFun = 1*costFun0 + 1*gammaTilde; % hard (same as soft) %%% Play with this
end

%% Solve the LMI problem (47)

solverOptions = sdpsettings('solver', 'mosek', 'verbose', 1);
sol = optimize(cons,[costFun],solverOptions);
statusGlobalController = sol.problem == 0;   



%%%% Comment fix the following
%% Extract variable values
gammaTildeVal = value(gammaTilde)
QVal = value(Q);
X_p_11Val = value(X_p_11);
KVal = X_p_11Val \ QVal;

% Load KVal elements into a cell structure K{i,j} (i.e., partitioning KVal
% into N\times N blocks)



% Filter the K_ij values (weed out the ones with the smallest magnitudes)


% Load the K_ij values to the DGs
for i = 1:1:numOfDGs
    for j = 1:1:numOfDGs
        DG{i}.K{j} = K{i,j};  
    end
end


%             costFun0Val = value(costFun0);
%             costFunVal = value(costFun);
%             PVal = value(P);
%             QVal = value(Q);
%             X_p_11Val = value(X_p_11);
%             X_p_21Val = value(X_p_21);
% 
%             gammaSqVal = value(gammaSq);
% 
%             M_neVal = X_p_11Val\QVal;
%             
%             % Obtaining K_ij blocks
%             M_neVal(nullMatBlock==1) = 0;
%             maxNorm = 0;
%             for i = 1:1:N
%                 for j = 1:1:N
%                     K{i,j} = M_neVal(3*(i-1)+1:3*i , 3*(j-1)+1:3*j); % (i,j)-th (3 x 3) block
%                     normVal = max(max(abs(K{i,j})));
%                     if normVal>maxNorm 
%                         maxNorm = normVal;
%                     end
%                 end
%             end
%             
%             % filtering out extremely small interconnections
%             for i=1:1:N
%                 for j=1:1:N
%                     if i~=j
%                         if isSoft
%                             K{i,j}(abs(K{i,j})<0.0001*maxNorm) = 0;                       
%                         else
%                             if A(j+1,i+1)==0
%                                 K{i,j} = zeros(3);
%                             end
%                         end
%                     end
%                     
%                     K_ijMax = max(abs(K{i,j}(:)));
%                     K{i,j}(abs(K{i,j})<0.01*K_ijMax) = 0;
% 
%                 end
%             end












end