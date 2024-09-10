%% Function to Check Local Feasibility Using Stored Passivity Indices
function plotLocalFeasibility(DG, Line, B_il, BarGamma, feasibleIndices, numOfDGs, numOfLines)
    % Create figure for local feasibility
    figure;
    hold on;
    title('Local Controller Feasibility');
    xlabel('log10(piScalar)');
    ylabel('log10(plScalar)');
    
    % Loop through each feasible combination of piScalar and plScalar
    for idx = 1:size(feasibleIndices, 1)
        piScalar = feasibleIndices(idx, 1);
        plScalar = feasibleIndices(idx, 2);

        % Define piVals and plVals using the feasible indices
        piVals = piScalar * ones(1, numOfDGs);
        plVals = plScalar * ones(1, numOfLines);

        try
            % Call the centralized local control design function
            [~, ~, statusLocalController] = ...
                centralizedLocalControlDesign(DG, Line, B_il, BarGamma, piVals, plVals);

            % Check local controller status and plot
            if statusLocalController == 1
                plot(log10(piScalar), log10(plScalar), 'bo', 'MarkerSize', 8, 'LineWidth', 1.5); % Blue circles for feasible local controllers
            else
                plot(log10(piScalar), log10(plScalar), 'ro', 'MarkerSize', 8, 'LineWidth', 1.5); % Red circles for non-feasible local controllers
            end
        catch ME
            % If an error occurs, skip to the next iteration
            continue;
        end
    end

    hold off;
    grid on;
    legend('Local Feasible', 'Local Non-Feasible');  % Legend for plot
end
