function plotCombinedControlDesignVariation(DG0, Line0, B_il, BarGamma, A_ij, isSoft, numOfDGs, numOfLines)
    % Define the step size and ranges for P_DG and P_Line
    step_size = 0.5;
    P_DG_Range = 10.^(-5:step_size:5); % Range for P_DG
    P_Line_Range = 10.^(-5:step_size:5); % Range for P_Line

    % Initialize the plot
    figure;
    hold on;

    % Loop through each combination of P_DG and P_Line
    for i= 1:length(P_DG_Range)
        for j = 1:length(P_Line_Range)

            P_DG = P_DG_Range(i);
            P_Line = P_Line_Range(j);

            % Calculate deltaP_DG and deltaP_Line based on the step size
            % deltaP_DG = 0;
            % deltaP_Line = 0; 
           
            deltaP_DG = 10^(P_DG + step_size) - 10^(P_DG - step_size);
            deltaP_Line = 10^(P_Line + step_size) - 10^(P_Line - step_size);

            % Define piVals and plVals with random variations
            piVals = P_DG + deltaP_DG * rand(1, numOfDGs) - 1/2 * deltaP_DG;
            plVals = P_Line + deltaP_Line * rand(1, numOfLines) - 1/2 * deltaP_Line;

            % Check feasibility using the provided function
            feasibilityStatus = checkFeasibility(piVals, plVals, DG0, Line0, B_il, BarGamma, A_ij, isSoft);

            % Plot the results based on the feasibility status
            if feasibilityStatus.local == 1
                plot(log10(P_DG), log10(P_Line), 'bx', 'MarkerSize', 8, 'LineWidth', 1.5);
            else
                plot(log10(P_DG), log10(P_Line), 'rx', 'MarkerSize', 8, 'LineWidth', 1.5);
            end

            if feasibilityStatus.global == 1
                plot(log10(P_DG), log10(P_Line), 'bo', 'MarkerSize', 8, 'LineWidth', 1.5);
            else
                plot(log10(P_DG), log10(P_Line), 'ro', 'MarkerSize', 8, 'LineWidth', 1.5);
            end
        end
    end

    % Customize the plot
    xlabel('log10(P_DG)');
    ylabel('log10(P_Line)');
    title('Combined Controller Status Across P_DG and P_Line');
    grid on;
    hold off;
end

function feasibilityStatus = checkFeasibility(piVals, plVals, DG0, Line0, B_il, BarGamma, A_ij, isSoft)
    try
        % Call the centralizedLocalControlDesign function
        [DG, Line, statusLocalController] = centralizedLocalControlDesign(DG0, Line0, B_il, BarGamma, piVals, plVals);

        % Initialize the feasibility status structure
        feasibilityStatus.local = statusLocalController;
        feasibilityStatus.global = 0;

        if statusLocalController == 1
            % Call the globalControlDesign function
            [DG, Line, statusGlobalController, ~, ~, ~, ~, ~, ~] = globalControlDesign(DG, Line, A_ij, B_il, BarGamma, isSoft);

            % Update global feasibility status
            feasibilityStatus.global = statusGlobalController;
        end
    catch
        % If an error occurs, set both statuses to 0 (not feasible)
        feasibilityStatus.local = 0;
        feasibilityStatus.global = 0;
    end
end
