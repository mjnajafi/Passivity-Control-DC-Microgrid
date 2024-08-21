function plotControlDesign(DG, Line, B_il, BarGamma, A_ij, isSoft, numOfDGs, numOfLines)
    % Define the correct range for piScalar and plScalar
    piRange = 10.^(-15:1:0);  % 10^-15 to 10^0
    plRange = 10.^(-15:1:0);  % 10^-15 to 10^0
    
    % Create a figure for the plot
    figure;
    hold on;
    
    % Dummy handles for the legend
    h1 = plot(NaN, NaN, 'bx', 'MarkerSize', 8, 'LineWidth', 1.5);  % Blue cross for Status 1
    h2 = plot(NaN, NaN, 'rx', 'MarkerSize', 5, 'LineWidth', 1);    % Red cross for Status 0
    
    % Loop through each combination of piScalar and plScalar
    for i = 1:length(piRange)
        for j = 1:length(plRange)
            piScalar = piRange(i);
            plScalar = plRange(j);
            
            % Define piVals and plVals
            piVals = piScalar * ones(1, numOfDGs);
            plVals = plScalar * ones(1, numOfLines);
            
            % Call the centralizedLocalControlDesign function
            [DG, Line, statusLocalController] = centralizedLocalControlDesign(DG, Line, B_il, BarGamma, piVals, plVals);
            
            % Call the globalControlDesign function
            [DG, Line, statusGlobalController, gammaTildeVal, K, C, BarC, H] = globalControlDesign(DG, Line, A_ij, B_il, BarGamma, isSoft);
            
            % Plot the result with a cross marker based on global controller status
            if statusGlobalController == 1
                % Blue cross for status 1
                plot(log10(piScalar), log10(plScalar), 'bx', 'MarkerSize', 8, 'LineWidth', 1.5);
            else
                % Red small cross for status 0
                plot(log10(piScalar), log10(plScalar), 'rx', 'MarkerSize', 5, 'LineWidth', 1);
            end
        end
    end
    
    % Customize the plot
    xlabel('log10(piScalar)');
    ylabel('log10(plScalar)');
    title('Global Controller Status Across piScalar and plScalar');
    grid on;
    legend([h1, h2], 'Status 1', 'Status 0');  % Use the dummy handles for the legend
    hold off;
end
