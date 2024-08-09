clc;
clear all;
close all;

% Define parameter ranges
R0_vals = [0.01, 0.02, 0.05, 0.1]; % Resistance values
L0_vals = [10e-6, 20e-6, 50e-6, 100e-6]; % Inductance values
C0_vals = [1e-3, 5e-3, 10e-3, 20e-3]; % Capacitance values
RL0_vals = [0.01, 0.1, 0.5, 1]; % Load resistance values
Y0_vals = 1 ./ RL0_vals; % Load conductance values
Rl0_vals = [0.01, 0.05, 0.1, 0.2]; % Line resistance values
Ll0_vals = [10e-6, 20e-6, 50e-6, 100e-6]; % Line inductance values

% Initialize variables to store the best results
minCost = Inf;
bestParams = struct();

numOfDGs = 5;
threshold = 0.6;
[coords, A_ij, B_il] = generateMicrogridTopology(numOfDGs, threshold);
minCost = 100000;
numOfDGs = size(B_il,1);
numOfLines = size(B_il,2);

BarGamma = 5;    % Fixed value for gammaBar
isSoft = 1; % Set to 1 to avoid the hard graph constraint

% pScalar = 1/(numOfDGs+numOfLines)
pScalar = 1;
piVals = pScalar*ones(1,numOfDGs);
plVals = pScalar*ones(1,numOfLines);

% Loop through all combinations of parameters
for R0 = R0_vals
    for L0 = L0_vals
        for C0 = C0_vals
            for RL0 = RL0_vals
                Y0 = 1 / RL0; % Update Y0 based on RL0
                for Rl0 = Rl0_vals
                    for Ll0 = Ll0_vals
                        % Set parameter values
                        % Update DG, Line, B_il, A_ij, and other relevant variables
                        % according to these parameter values

                        for i = 1:1:numOfDGs
                            DG{i} = GenerateDG(R0,L0,C0,RL0,coords(i,:));
                        end

                        for l = 1:1:numOfLines
                            Line{l} = GenerateLine(Rl0,Ll0);
                        end

                        % Call the control design functions
                        [DG, Line, statusLocalController] = centralizedLocalControlDesign(DG, Line, B_il, BarGamma, piVals, plVals);
                        % [DG, Line, statusGlobalController, gammaTildeVal] = globalControlDesign(DG, Line, A_ij, B_il, BarGamma, isSoft);

                        % Compute the cost
                        % cost = gammaTildeVal;
                        % Uncomment and modify if you want to include penalty terms
                        % cost = gammaTildeVal + 10000*(1-statusLocalController) + 10000*(1-statusGlobalController);

                        % Check feasibility
                        con1 = statusLocalController == 1;
                        % con2 = statusGlobalController == 1;

                        % Update the best result if this combination is feasible and has lower cost
                        % if statusGlobalController == 1 && con1 && con2
                        if con1
                            disp(['R0: ', num2str(R0)]);
                            disp(['L0: ', num2str(L0)]);
                            disp(['C0: ', num2str(C0)]);
                            disp(['RL0: ', num2str(RL0)]);
                            disp(['Y0: ', num2str(Y0)]);
                            disp(['Rl0: ', num2str(Rl0)]);
                            disp(['Ll0: ', num2str(Ll0)]);
                            
                            % if cost < minCost
                            %     minCost = cost;
                            %     bestParams.R0 = R0;
                            %     bestParams.L0 = L0;
                            %     bestParams.C0 = C0;
                            %     bestParams.RL0 = RL0;
                            %     bestParams.Y0 = Y0;
                            %     bestParams.Rl0 = Rl0;
                            %     bestParams.Ll0 = Ll0;
                            % end
                        end
                    end
                end
            end
        end
    end
end

% Display the best parameters
disp('Best parameters:');
disp(bestParams);
