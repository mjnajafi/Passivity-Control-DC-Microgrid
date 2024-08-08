function [C,Ceq] = centralizedCodesignFeasibility(pVals)
    %%$ Comment: Complete this function
N = 10;

% Temorary
statusL = ones(N,1);
statusG = 1;

C = [statusL-ones(N,1); statusG-1]; % All local and global controllers need to be feasible
Ceq = [];

%% Example codes from platooning problem
% function [C,Ceq] = centralizedRobustControllerSynthesis2CodesignFeasibility(obj,pVals)
% Number of follower vehicles
% N = obj.numOfVehicles-1; 
% 
% % Load local controllers and resulting passivity indices
% statusLVals = [];
% LVals = [];
% nuVals = [];
% rhoVals = [];
% gammaSqVals = [];
% for i = 1:1:N
%     p_i = pVals(i);
%     [statusL_i,PVal,KVal,LVal,nuVal,rhoVal,gammaSqVal] = obj.vehicles(i+1).synthesizeLocalControllersParameterized(2,p_i);
% 
%     statusLVals = [statusLVals;statusL_i];
%     LVals = [LVals;LVal];
%     nuVals = [nuVals,nuVal];
%     rhoVals = [rhoVals,rhoVal];
%     gammaSqVals = [gammaSqVals,gammaSqVal];
% 
% end
% 
% if any(statusLVals==0)
%     statusG = 0;
%     statusK = norm(LVals) <= 1000*sqrt(N);
%     C = [statusLVals-ones(N,1); statusG-1; statusK-1];
%     Ceq = [];
%     return;
% end
% % Else: continue to do the gobal design
% 
% % Creating the adgacency matrix, null matrix and cost matrix
% G = obj.topology.graph;
% A = adjacency(G);
% for i = 1:1:N
%     for j = 1:1:N
%         % Structure of K_ij (which is a 3x3 matrix) should be embedded here
%         if i~=j
%             if A(j+1,i+1)==1
%                 adjMatBlock{i,j} = [0,0,0; 0,0,0; 1,1,1];
%                 nullMatBlock{i,j} = [1,1,1; 1,1,1; 0,0,0];
%                 costMatBlock{i,j} = 1*[0,0,0; 0,0,0; 1,1,1];
%             else
%                 adjMatBlock{i,j} = [0,0,0; 0,0,0; 0,0,0];
%                 nullMatBlock{i,j} = [1,1,1; 1,1,1; 0,0,0];
%                 costMatBlock{i,j} = (20/N)*abs(i-j)*[0,0,0; 0,0,0; 1,1,1];
%             end
%         else
%             adjMatBlock{i,j} = [0,0,0; 0,0,0; 1,1,1];
%             nullMatBlock{i,j} = [1,1,1; 1,1,1; 0,0,0];
%             costMatBlock{i,j} = 0*[0,0,0; 0,0,0; 1,1,1];
%         end
%     end 
% end
% adjMatBlock = cell2mat(adjMatBlock);
% nullMatBlock = cell2mat(nullMatBlock);
% costMatBlock = cell2mat(costMatBlock);
% 
% % Set up the LMI problem
% solverOptions = sdpsettings('solver','mosek','verbose',0);
% I = eye(3*N);
% I_n = eye(3);
% O = zeros(3*N);
% 
% % Whether to use a soft or hard graph constraint
% isSoft = 1;
% % normType = 2;
% minCostVal = 0.004;
% 
% Q = sdpvar(3*N,3*N,'full'); 
% P = sdpvar(N,N,'diagonal');
% gammaSq = sdpvar(1,1,'full');
% 
% X_p_11 = [];
% X_p_12 = [];
% X_12 = [];
% X_p_22 = [];
% for i = 1:1:N
% %                 nu_i = obj.vehicles(i+1).nu;
% %                 rho_i = obj.vehicles(i+1).rho;
%     nu_i = nuVals(i);
%     rho_i = rhoVals(i);
%     X_p_11 = blkdiag(X_p_11,-nu_i*P(i,i)*I_n);
%     X_p_12 = blkdiag(X_p_12,0.5*P(i,i)*I_n);
%     X_12 = blkdiag(X_12,(-1/(2*nu_i))*I_n);
%     X_p_22 = blkdiag(X_p_22,-rho_i*P(i,i)*I_n);
% end
% X_p_21 = X_p_12';
% X_21 = X_12';
% 
% % Objective Function
% costFun0 = sum(sum(Q.*costMatBlock));
% % costFun0 = norm(Q.*costMatBlock,2);
% 
% % Minimum Budget Constraints
% con0 = costFun0 >= minCostVal;
% 
% % Basic Constraints
% con1 = P >= 0;
% 
% DMat = [X_p_11, O; O, I];
% MMat = [Q, X_p_11; I, O];
% ThetaMat = [-X_21*Q-Q'*X_12-X_p_22, -X_p_21; -X_p_12, gammaSq*I];
% con2 = [DMat, MMat; MMat', ThetaMat] >= 0; % The real one
% 
% % Structural constraints
% con3 = Q.*(nullMatBlock==1)==O;  % Structural limitations (due to the format of the control law)
% con4 = Q.*(adjMatBlock==0)==O;  % Graph structure : hard constraint
% 
% 
% % Total Cost and Constraints
% if isSoft
%     cons = [con0,con1,con2,con3]; % Without the hard graph constraint con7
%     costFun = 1*costFun0 + 1*gammaSq; % soft 
% else
%     cons = [con0,con1,con2,con3,con4]; % With the hard graph constraint con7
%     costFun = 1*costFun0 + 1*gammaSq; % hard (same as soft)
% end
% 
% sol = optimize(cons,[costFun],solverOptions);
% statusG = sol.problem == 0; %sol.info;
% 
% statusK = norm(LVals) <= 1000*sqrt(N);
% C = [statusLVals-ones(N,1); statusG-1; statusK-1];
% Ceq = [];
% end
end

