%% Function to Check Global Feasibility and Store Passivity Indices
function feasibleIndices = checkGlobalFeasibility(DG, Line, B_il, BarGamma, A_ij, isSoft, numOfDGs, numOfLines)
    % Define the ranges for piScalar and plScalar
    piRange = 10.^(-5:0.5:0);  % Range for piScalar
    plRange = 10.^(-5:0.5:0);  % Range for plScalar

    feasibleIndices = [];  % Initialize an array to hold feasible combinations

    % Create figure for global feasibility
    figure;
    hold on;
    title('Global Controller Feasibility');
    xlabel('log10(piScalar)');
    ylabel('log10(plScalar)');

    % Loop through each combination of piScalar and plScalar
    for i = 1:length(piRange)
        for j = 1:length(plRange)
            piScalar = piRange(i);
            plScalar = plRange(j);

            % Define piVals and plVals
            piVals = piScalar * ones(1, numOfDGs);
            plVals = plScalar * ones(1, numOfLines);

            try
                % Call the global control design function
                [~, ~, statusGlobalController, ~, ~, ~, ~, ~, ~] = ...
                    globalControlDesign(DG, Line, A_ij, B_il, BarGamma, isSoft);

                % If global is feasible, store the current piScalar and plScalar
                if statusGlobalController == 1
                    feasibleIndices = [feasibleIndices; piScalar, plScalar];  % Store the indices
                    plot(log10(piScalar), log10(plScalar), 'bo', 'MarkerSize', 8, 'LineWidth', 1.5);  % Green circles for feasible global controllers
                else
                    plot(log10(piScalar), log10(plScalar), 'ro', 'MarkerSize', 8, 'LineWidth', 1.5);  % Red circles for non-feasible global controllers
                end
            catch ME
                % If an error occurs, skip to the next iteration
                continue;
            end
        end
    end

    hold off;
    grid on;
    legend('Global Feasible', 'Global Non-Feasible');  % Legend for plot
    % Display feasible indices if any
    if ~isempty(feasibleIndices)
        disp('Feasible indices (piScalar, plScalar):');
        disp(feasibleIndices);
    else
        disp('No feasible combinations found.');
    end
end
