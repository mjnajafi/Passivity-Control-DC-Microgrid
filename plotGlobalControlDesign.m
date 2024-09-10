% First Method
function plotGlobalControlDesign(DG, Line, B_il, BarGamma, A_ij, isSoft, numOfDGs, numOfLines)

    % Turn off all warnings
    warning off;
    % Define the correct range for piScalar and plScalar
    piRange = 10.^(-10:1:10);  % 10^-15 to 10^0
    plRange = 10.^(-10:1:10);  % 10^-15 to 10^0

    % Create a figure for the plot
    figure;
    hold on;

    % Dummy handles for the legend
    h1 = plot(NaN, NaN, 'bx', 'MarkerSize', 8, 'LineWidth', 1.5);  % Blue cross for Status 1
    h2 = plot(NaN, NaN, 'rx', 'MarkerSize', 5, 'LineWidth', 1);    % Red cross for Status 0

    DG0 = DG;
    Line0 = Line;

    % Loop through each combination of piScalar and plScalar
    for i = 1:length(piRange)
        for j = 1:length(plRange)
            piScalar = piRange(i);
            plScalar = plRange(j);

            % Define piVals and plVals
            piVals = piScalar * ones(1, numOfDGs);
            plVals = plScalar * ones(1, numOfLines);

            % Call the centralizedLocalControlDesign function
            [DG, Line, statusLocalController] = centralizedLocalControlDesign(DG0, Line0, B_il, BarGamma, piVals, plVals);

            try
                % Call the globalControlDesign function
                [~, ~, statusGlobalController, ~, ~, ~, ~, ~] = globalControlDesign(DG, Line, A_ij, B_il, BarGamma, isSoft);

                % Plot the result with a cross marker based on global controller status
                if statusGlobalController == 1
                    % Blue cross for status 1
                    plot(log10(piScalar), log10(plScalar), 'bx', 'MarkerSize', 8, 'LineWidth', 1.5);
                else
                    % Red small cross for status 0
                    plot(log10(piScalar), log10(plScalar), 'rx', 'MarkerSize', 5, 'LineWidth', 1);
                end
            catch ME
                % If an error occurs, display a warning and skip to the next iteration
                warning('Error encountered with piScalar = %e and plScalar = %e: %s', piScalar, plScalar, ME.message);
                continue;
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
% %% Second Method
% function plotCombinedControlDesign(DG, Line, B_il, BarGamma, A_ij, isSoft, numOfDGs, numOfLines)
%     % Turn off all warnings
%     warning off;
% 
%     % Define the correct range for piScalar and plScalar
%     piRange = 10.^(-5:.5:5);  % Adjust the range as needed
%     plRange = 10.^(-5:.5:5);  % Adjust the range as needed
% 
%     % Create a figure for the plot
%     figure;
%     hold on;
% 
%     % Dummy handles for the legend
%     h1 = plot(NaN, NaN, 'bx', 'MarkerSize', 8, 'LineWidth', 1.5);  % Blue cross for all feasibilities 1
%     h2 = plot(NaN, NaN, 'rx', 'MarkerSize', 5, 'LineWidth', 1);    % Red cross if any feasibility is 0
% 
% 
% 
%     % Loop through each combination of piScalar and plScalar
%     for i = 1:length(piRange)
%         for j = 1:length(plRange)
%             piScalar = piRange(i);
%             plScalar = plRange(j);
% 
%             % Define piVals and plVals
%             piVals = piScalar * ones(1, numOfDGs);
%             plVals = plScalar * ones(1, numOfLines);
% 
%             % Call the designDGPassivity function
%             [DG, rhoTilde_i, nu_i, gammaTilde_i, statusDG] = designDGPassivity(DG, B_il, piVals);
% 
%             % Call the designLinePassivity function
%             [Line, statusLine] = designLinePassivity(Line, DG, B_il, BarGamma, piVals, plVals, rhoTilde_i, nu_i);
% 
%             [DG, Line, statusGlobalController, gammaTildeVal, K, C, BarC, H] = globalControlDesign(DG, Line, A_ij, B_il, BarGamma, isSoft);
% 
%             try
%                 % Call the globalControlDesign function
%                 % [DG, Line, statusGlobalController, gammaTildeVal, K, C, BarC, H] = globalControlDesign(DG, Line, A_ij, B_il, BarGamma, isSoft);
% 
%                 % Plot based on global controller feasibility
%                 if statusGlobalController == 1
%                     % Blue cross if all feasibilities are 1
%                     plot(log10(piScalar), log10(plScalar), 'bx', 'MarkerSize', 8, 'LineWidth', 1.5);
%                 else
%                     % Red cross if global controller feasibility is 0
%                     plot(log10(piScalar), log10(plScalar), 'rx', 'MarkerSize', 8, 'LineWidth', 1.5);
%                 end
%             catch ME
%                 % Handle any errors in global control design
%                 % Skip to the next iteration
%                 continue;
%             end
%         end
%     end
% 
%     % Customize the plot
%     xlabel('log10(piScalar)');
%     ylabel('log10(plScalar)');
%     title('Combined Controller Feasibility Across piScalar and plScalar');
%     grid on;
%     legend([h1, h2], 'All Feasibilities 1', 'Any Feasibility 0');  % Use the dummy handles for the legend
%     hold off;
% 
%     % Turn warnings back on
%     warning on;
% end
