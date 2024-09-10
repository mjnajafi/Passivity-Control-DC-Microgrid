clear all
close all
clc

count = 0;
N = 50;
C0Array = logspace(log10(220), log10(0.022), N);  % N is the number of steps you want

for C0 = C0Array
    close all
    count = count + 1
    
    rng(7)
    
    numOfDGs = 4;
    threshold = 0.6;
    [coords, A_ij, B_il] = generateMicrogridTopology(numOfDGs, threshold);
    
    
    % Initial parameter values for DGs
    R0 = 0.02;      % Resistance
    L0 = 0.01;      % Inductance
    % C0 = 22;    % Capacitance
    RL0 = 0.3;      % Load Resistance
    Y0 = 1/RL0;     % Load Conductance
    IL0 = 5;        % Constant Current Load
    LoadNoiseMean = 10;
    LoadNoiseStd = 2;
    
    
    for i = 1:1:numOfDGs
        DG{i} = GenerateDG(R0,L0,C0,RL0,IL0,coords(i,:));
        
        [PVal, KVal, LVal, nuVal, rhoVal, status] = ComputePassivityForDGs(DG{i});
    end
    
    % Initial parameter values for Lines
    Rl0 = 2;     % Line Resistance
    Ll0 = 0.01;     % Line Inductance
    
    numOfLines = size(B_il,2);
    
    for l = 1:1:numOfLines
        Line{l} = GenerateLine(Rl0,Ll0);
        [PBarVal, nuBarVal, rhoBarVal, status] = ComputePassivityForLines(Line{l});
    end
    BarGamma = 1000000;    % Fixed value for gammaBar
    isSoft = 1;      % Set to 1 to avoid the hard graph constraint
    
    plotCombinedControlDesign(DG, Line, B_il, BarGamma, A_ij, isSoft, numOfDGs, numOfLines);
    print(['Results/CResults/C0_',num2str(count)], '-dpng', '-r600')
end