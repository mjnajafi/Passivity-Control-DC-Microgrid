% This script simulates the control design for a microgrid consisting of
% Distributed Generators (DGs) and transmission lines. It generates a 
% microgrid topology, initializes parameters for DGs and lines, and 
% evaluates control strategies based on a range of scalar values.

clc;
clear all; 
close all;


numOfDGs = 5;
threshold = 0.6;
[coords, A_ij, B_il] = generateMicrogridTopology(numOfDGs, threshold);
minCost = 100000;
numOfDGs = size(B_il,1);
numOfLines = size(B_il,2);

% Initial parameter values for DGs
R0 = 0.02;      % Resistance
L0 = 0.01;      % Inductance
C0 = 0.0022;    % Capacitance
RL0 = 0.3;      % Load Resistance
Y0 = 1/RL0;     % Load Conductance

% Initial parameter values for Lines
Rl0 = 0.02;     % Line Resistance
Ll0 = 0.01;     % Line Inductance

for i = 1:1:numOfDGs
        DG{i} = GenerateDG(R0,L0,C0,RL0,coords(i,:));
    end

    for l = 1:1:numOfLines
        Line{l} = GenerateLine(Rl0,Ll0);
    end

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
