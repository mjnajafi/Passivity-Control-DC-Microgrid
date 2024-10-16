function plotCombinedControlDesignVariation(DG0, Line0, B_il, BarGamma, A_ij, isSoft, numOfDGs, numOfLines)
    % Define the step size for linear variation
    step_size = 1;
    P_DG_Range = -5:step_size:5; % Range for P_DG in linear scale
    P_Line_Range = -5:step_size:5; % Range for P_Line in linear scale

    % Initialize the plot
    figure;
    hold on;

    % Loop through each combination of P_DG and P_Line
    for i = 1:length(P_DG_Range)
        for j = 1:length(P_Line_Range)

            P_DG = 10^P_DG_Range(i); % Convert back to original scale
            P_Line = 10^P_Line_Range(j); % Convert back to original scale

            % deltaP_DG = 10^(P_DG + step_size) - 10^(P_DG - step_size);
            % deltaP_Line = 10^(P_Line + step_size) - 10^(P_Line - step_size);

            % deltaP_DG = log10(10^(P_DG + step_size)) - log10(10^(P_DG - step_size));
            % deltaP_Line = log10(10^(P_Line + step_size)) - log10(10^(P_Line - step_size));
            % 
            % 
            % deltaP_DG = step_size * (10^(P_DG) * log10(10));
            % deltaP_Line = step_size * (10^(P_Line) * log10(10));
            % 
            % deltaP_DG = step_size * log10(10) * 10^P_DG;  % Linear approximation
            % deltaP_Line = step_size * log10(10) * 10^P_Line;  % Linear approximation

            % % Define linear delta values
            % deltaP_DG = step_size; % Linear change for P_DG
            % deltaP_Line = step_size; % Linear change for P_Line

            P_DG_Low = 10^(0.5*(P_DG_Range(i)+P_DG_Range(i-1));
            P_Line_Low = P_Line - 0.5*deltaP_Line;

            % Vertical line
            plot([P_DG_Low, 10^P_Line_Range(1)], [P_DG_Low, 10^P_Line_Range(end)], '-k', 'MarkerSize', 8, 'LineWidth', 1.5);
            % Horizontal line
            plot([10^P_DG_Range(1), P_Line_Low], [10^P_DG_Range(end), P_Line_Low,], '-k', 'MarkerSize', 8, 'LineWidth', 1.5);

            % Define piVals and plVals with linear variations
            piVals = P_DG + F(rand(1, numOfDGs)); % Find F such that piVals will be in the correct interval [P_DG_Low,P_DG_high]. left 
            % P_DG_Low is the x coordinate of left line in the box, P_DP is
            % the center of the box. 
            % Generate 100 values from the uniform distribution on the interval (a, b).
            % r = a + (b-a).*rand(100,1);
            % Generate 100 values from a normal distribution with mean 1 and standard deviation 2.
            % r = 1 + 2.*randn(100,1);
 

            plVals = P_Line + deltaP_Line * rand(1, numOfLines) - 1/2 * deltaP_Line;

            % Initialize the feasibility status structure
            feasibilityStatus.local = 0;
            feasibilityStatus.global = 0;

            try
                % Call the centralizedLocalControlDesign function
                [DG, Line, statusLocalController] = centralizedLocalControlDesign(DG0, Line0, B_il, BarGamma, piVals, plVals);

                % Update local feasibility status
                feasibilityStatus.local = statusLocalController;

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
    xlabel('log10(P_{DG})');
    ylabel('log10(P_{Line})');
    title('Combined Controller Status Across P_DG and P_Line');
    grid on;
    hold off;
end
