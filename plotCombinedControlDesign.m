%% First Method
% function plotCombinedControlDesign(DG, Line, B_il, BarGamma, A_ij, isSoft, numOfDGs, numOfLines)
%     % Turn off all warnings
%     warning off;
% 
%     % Define the correct range for piScalar and plScalar
%     piRange = 10.^(-10:.5:5);  % Adjust the range as needed
%     plRange = 10.^(-10:.5:5);  % Adjust the range as needed
% 
%     % Create a figure for the plot
%     figure;
%     hold on;
% 
%     % Dummy handles for the legend
%     h1 = plot(NaN, NaN, 'bx', 'MarkerSize', 8, 'LineWidth', 1.5);  % Blue cross for both feasibilities 1
%     h2 = plot(NaN, NaN, 'rx', 'MarkerSize', 5, 'LineWidth', 1);    % Red cross if any feasibility is 0
% 
%     DG0 = DG;
%     Line0 = Line;
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
%             % Call the centralizedLocalControlDesign function
%             [DG, Line, statusLocalController] = centralizedLocalControlDesign(DG0, Line0, B_il, BarGamma, piVals, plVals);
% 
%             try
%                 % Call the globalControlDesign function
%                 [DG, Line, statusGlobalController, gammaTildeVal, K, C, BarC, H] = globalControlDesign(DG, Line, A_ij, B_il, BarGamma, isSoft);
% 
%                 % Plot a cross only if both feasibilities occur
%                 if statusLocalController == 1 && statusGlobalController == 1
%                     % Blue cross if both statuses are 1
%                     plot(log10(piScalar), log10(plScalar), 'bx', 'MarkerSize', 8, 'LineWidth', 1.5);
%                 else
%                     % Red cross if either status is 0
%                     plot(log10(piScalar), log10(plScalar), 'rx', 'MarkerSize', 8, 'LineWidth', 1.5);
%                 end
%             catch ME
%                 % If an error occurs, skip to the next iteration
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
%     legend([h1, h2], 'Both Feasibilities 1', 'Any Feasibility 0');  % Use the dummy handles for the legend
%     hold off;
% 
%     % Turn warnings back on
%     warning on;
% end
%% Second Method
function plotCombinedControlDesign(DG, Line, B_il, BarGamma, A_ij, isSoft, numOfDGs, numOfLines)
    % Turn off all warnings
    warning off;

    % Define the correct range for piScalar and plScalar
    piRange = 10.^(-10:1:10);  % Adjust the range as needed
    plRange = 10.^(-10:1:10);  % Adjust the range as needed

    % Create a figure for the plot
    figure;
    hold on;

    % Dummy handles for the legend
    h1 = plot(NaN, NaN, 'bx', 'MarkerSize', 8, 'LineWidth', 1.5);  % Blue cross for both Status 1
    h2 = plot(NaN, NaN, 'rx', 'MarkerSize', 5, 'LineWidth', 1);    % Red cross for any Status 0

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
            % if statusLocalController == 1

            % Plot the result based on the combined status
            if statusLocalController == 1
                % Blue cross for local controller status 1
                plot(log10(piScalar), log10(plScalar), 'bx', 'MarkerSize', 8, 'LineWidth', 1.5);
            else 
                % Red cross if local controller status is 0
                plot(log10(piScalar), log10(plScalar), 'rx', 'MarkerSize', 8, 'LineWidth', 1.5);
            end

            try
                % Call the globalControlDesign function
                [~,~, statusGlobalController, ~,~,~,~,~,~,~] = globalControlDesign(DG, Line, A_ij, B_il, BarGamma, isSoft);
                
                if statusGlobalController == 1
                    
                    % Blue circle if global controller status is 1
                    plot(log10(piScalar), log10(plScalar), 'bo', 'MarkerSize', 8, 'LineWidth', 1.5);
                    % plot(log10(mean(P_iVal)), log10(mean(P_lVal)), 'b*', 'MarkerSize', 8, 'LineWidth', 1.5);
                else
                    % Red circle if global controller status is 0
                    plot(log10(piScalar), log10(plScalar), 'ro', 'MarkerSize', 8, 'LineWidth', 1.5);
                    % plot(log10(mean(P_iVal)), log10(mean(P_lVal)), 'r*', 'MarkerSize', 8, 'LineWidth', 1.5);
                end
            catch ME
                % If an error occurs, skip to the next iteration
                continue;
            end
            % end
        end
    end

    % Customize the plot
    xlabel('log10(piScalar)');
    ylabel('log10(plScalar)');
    title('Combined Controller Status Across piScalar and plScalar');
    grid on;
    legend([h1, h2], 'Both Statuses 1', 'Either Status 0');  % Use the dummy handles for the legend
    hold off;

    % Turn warnings back on
    warning on;
end
% %% Third Method
% function plotCombinedControlDesign(DG, Line, B_il, BarGamma, A_ij, isSoft, numOfDGs, numOfLines)
%     % Turn off all warnings
%     warning off;
% 
%     % Define the correct range for piScalar and plScalar
%     piRange = 10.^(-5:.5:10);  % Adjust the range as needed
%     plRange = 10.^(-5:.5:10);  % Adjust the range as needed
% 
%     % Create a figure for the plot
%     figure;
%     hold on;
% 
%     % Dummy handles for the legend
%     h1 = plot(NaN, NaN, 'bx', 'MarkerSize', 8, 'LineWidth', 1.5);  % Blue cross for all statuses 1
%     h2 = plot(NaN, NaN, 'rx', 'MarkerSize', 5, 'LineWidth', 1);    % Red cross if any status is 0
% 
%     DG0 = DG;
%     Line0 = Line;
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
%             [DG, rhoTilde_i, nu_i, gammaTilde_i, statusDG] = designDGPassivity(DG0, B_il, piVals);
% 
%             % Call the designLinePassivity function
%             [Line, statusLine] = designLinePassivity(Line0, DG, B_il, BarGamma, piVals, plVals, rhoTilde_i, nu_i);
% 
%             try
%                 % Call the globalControlDesign function
%                 [DG, Line, statusGlobalController, gammaTildeVal, K, C, BarC, H] = globalControlDesign(DG, Line, A_ij, B_il, BarGamma, isSoft);
% 
%                 % Plot a cross only if all three statuses are 1
%                 if statusDG == 1 && statusLine == 1 && statusGlobalController == 1
%                     % Blue cross if all statuses are 1
%                     plot(log10(piScalar), log10(plScalar), 'bx', 'MarkerSize', 8, 'LineWidth', 1.5);
%                 else
%                     % Red cross if any status is 0
%                     plot(log10(piScalar), log10(plScalar), 'rx', 'MarkerSize', 8, 'LineWidth', 1.5);
%                 end
%             catch ME
%                 % If an error occurs, skip to the next iteration
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
