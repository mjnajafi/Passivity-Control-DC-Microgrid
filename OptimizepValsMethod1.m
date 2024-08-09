% minCost = 100000;
% 
% for pScalar = 0.01:0.1:1
% 
%     BarGamma = 5;    % Fixed value for gammaBar
%     isSoft = 1; % Set to 1 to avoid the hard graph constraint
% 
%     piVals = pScalar*ones(1,numOfDGs)
%     plVals = pScalar*ones(1,numOfLines)
% 
%     [DG,Line,statusLocalController] = centralizedLocalControlDesign(DG,Line,B_il,BarGamma,piVals,plVals); % topologyMetrics may be the B matrix 
%     [DG,Line,statusGlobalController,gammaTildeVal] = globalControlDesign(DG,Line,A_ij,B_il,BarGamma,isSoft);
% 
%     cost = gammaTildeVal;
%     % cost = gammaTildeVal + 10000*(1-statusLocalController) + 10000*(1-statusGlobalController) 
% 
%     con1 = statusLocalController == 1;
%     con2 = statusGlobalController == 1;
%     % con3 = norm(DG{1}.K0) < 1000;
% 
%     if cost<minCost && con1 && con2
%         minCost = cost;
%         pStar = pScalar;
%     end
% 
% end
% 
% % minCost
% % pStar
numOfDGs = 5;
threshold = 0.6;
[coords, A_ij, B_il] = generateMicrogridTopology(numOfDGs, threshold);
minCost = 100000;
numOfDGs = size(B_il,1);
numOfLines = size(B_il,2);

for pScalar = 0.01:0.01:10

    BarGamma = 5;    % Fixed value for gammaBar
    isSoft = 1; % Set to 1 to avoid the hard graph constraint

    piVals = pScalar*ones(1,numOfDGs);
    plVals = pScalar*ones(1,numOfLines);

    
    [DG,Line,statusLocalController] = centralizedLocalControlDesign(DG,Line,B_il,BarGamma,piVals,plVals); % topologyMetrics may be the B matrix 
    [DG,Line,statusGlobalController,gammaTildeVal] = globalControlDesign(DG,Line,A_ij,B_il,BarGamma,isSoft);

    cost = gammaTildeVal;
    % cost = gammaTildeVal + 10000*(1-statusLocalController) + 10000*(1-statusGlobalController) 

    con1 = statusLocalController == 1;
    con2 = statusGlobalController == 1;
    % con3 = norm(DG{1}.K0) < 1000;

    if con1
        disp(['pScalar: ', num2str(pScalar)]);
        disp(['piVals: ', num2str(piVals)]);
        disp(['plVals: ', num2str(plVals)]);
    end

    % if cost < minCost && con1 && con2
    %     minCost = cost;
    %     pStar = pScalar;
    % end

end

% minCost
% pStar
