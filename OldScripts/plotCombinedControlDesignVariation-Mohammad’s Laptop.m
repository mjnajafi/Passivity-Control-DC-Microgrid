function plotCombinedControlDesignVariation(DG0, Line0, B_il, BarGamma, A_ij, isSoft, numOfDGs, numOfLines)
    % Define the step size and ranges for P_DG and P_Line
    step_size = 0.5;
    P_DG_Range = 10.^(-5:step_size:5); % Range for P_DG
    P_Line_Range = 10.^(-5:step_size:5); % Range for P_Line

    % Initialize the plot
    figure;
    hold on;

    % Loop through each combination of P_DG and P_Line
    for i = 1:length(P_DG_Range)
        for j = 1:length(P_Line_Range)

            P_DG = P_DG_Range(i);
            P_Line = P_Line_Range(j);

            % Calculate deltaP_DG and deltaP_Line based on the step size
            deltaP_DG = 10^(P_DG + step_size) - 10^(P_DG - step_size);
            deltaP_Line = 10^(P_Line + step_size) - 10^(P_Line - step_size);

            % Define piVals and plVals with random variations
            piVals = P_DG + deltaP_DG * rand(1, numOfDGs) - 1/2 * deltaP_DG;
            plVals = P_Line + deltaP_Line * rand(1, numOfLines) - 1/2 * deltaP_Line;

            try
                % Call the centralizedLocalControlDesign function
                [DG, Line, statusLocalController] = centralizedLocalControlDesign(DG0, Line0, B_il, BarGamma, piVals, plVals);
                
                % Debugging information
                disp(['P_DG: ', num2str(P_DG), ', P_Line: ', num2str(P_Line)]);
                disp(['Local Status: ', num2str(statusLocalController)]);

                % Call the globalControlDesign function
                [DG, Line, statusGlobalController, gammaTildeVal, K, C, BarC, H, P_iVal, P_lVal] = globalControlDesign(DG, Line, A_ij, B_il, BarGamma, isSoft);
                
                % More debugging information
                disp(['Global Status: ', num2str(statusGlobalController)]);

                % Plot based on the local controller status
                if statusLocalController == 1
                    plot(log10(P_DG), log10(P_Line), 'bx', 'MarkerSize', 8, 'LineWidth', 1.5);
                else
                    plot(log10(P_DG), log10(P_Line), 'rx', 'MarkerSize', 8, 'LineWidth', 1.5);
                end

                % Plot based on the global controller status
                if statusGlobalController == 1
                    plot(log10(P_DG), log10(P_Line), 'bo', 'MarkerSize', 8, 'LineWidth', 1.5);
                else
                    plot(log10(P_DG), log10(P_Line), 'ro', 'MarkerSize', 8, 'LineWidth', 1.5);
                end

                pause(0.01); % Pause to update plot
            catch ME
                % If an error occurs, print the error and skip to the next iteration
                disp(['Error: ', ME.message]);
                continue;
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
