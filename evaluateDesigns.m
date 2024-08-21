% This function evaluates the feasibility of local and global control designs 
% for different values of numOfDGs, threshold, and pScalar.

function [numOfDGs, threshold] = evaluateDesigns(numDGsRange, thresholdRange, BarGamma, pScalarRange, isSoft);
    % Function to evaluate the feasibility of local and global control designs
    % for different values of numOfDGs, threshold, and pScalar.
    % Input:
    %   numDGsRange - Vector of possible numOfDGs values
    %   thresholdRange - Vector of possible threshold values
    %   BarGamma - Fixed value for gammaBar
    %   pScalarRange - Vector of possible pScalar values
    %   isSoft - Set to 1 for soft graph constraint, 0 otherwise

    % Loop over numOfDGs and threshold values
    for numOfDGs = numDGsRange
        for threshold = thresholdRange
            % Generate the microgrid topology
            [coords, A_ij, B_il] = generateMicrogridTopology(numOfDGs, threshold);
            numOfLines = size(B_il, 2);

            % Initialize DG and Line
            DG = cell(1, numOfDGs);
            Line = cell(1, numOfLines);

            % Initial parameter values for DGs
            R0 = 0.02;      % Resistance
            L0 = 0.01;      % Inductance
            C0 = 0.0022;    % Capacitance
            RL0 = 0.3;      % Load Resistance
            Y0 = 1/RL0;     % Load Conductance
            IL0 = 5;        % Constant Current Load

            % Initial parameter values for Lines
            Rl0 = 0.02;     % Line Resistance
            Ll0 = 0.01;     % Line Inductance

            for i = 1:numOfDGs
                DG{i} = GenerateDG(R0, L0, C0, RL0, IL0, coords(i, :));
            end

            for l = 1:numOfLines
                Line{l} = GenerateLine(Rl0, Ll0);
            end

            % Loop over pScalar values
            for pScalar = pScalarRange
                % Define piVals and plVals
                piVals = pScalar * ones(1, numOfDGs);
                plVals = pScalar * ones(1, numOfLines);

                % Design local controller
                [DG, Line, statusLocalController] = centralizedLocalControlDesign(DG, Line, B_il, BarGamma, piVals, plVals);

                % Design global controller
                [DG, Line, statusGlobalController, gammaTildeVal] = globalControlDesign(DG, Line, A_ij, B_il, BarGamma, isSoft);

                % Check constraints
                con1 = statusLocalController == 1;
                con2 = statusGlobalController == 1;

                if con1 && con2
                    % Display results if both controllers are feasible
                    fprintf('Feasible design found:\n');
                    fprintf('numOfDGs: %d\n', numOfDGs);
                    fprintf('threshold: %.2f\n', threshold);
                    fprintf('pScalar: %.2f\n', pScalar);
                    fprintf('piVals: %s\n', mat2str(piVals));
                    fprintf('plVals: %s\n', mat2str(plVals));
                    fprintf('gammaTildeVal: %.2f\n', gammaTildeVal);
                    fprintf('---------------------------------------------\n');
                end
            end
        end
    end
end
