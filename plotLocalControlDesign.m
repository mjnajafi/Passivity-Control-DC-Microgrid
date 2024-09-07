%% First Method
function plotLocalControlDesign(DG, Line, B_il, BarGamma, numOfDGs, numOfLines)

    % Turn off all warnings
    warning off;
    % Define the correct range for piScalar and plScalar
    piRange = 10.^(-10:1:10);  % 10^-15 to 10^0
    plRange = 10.^(-10:1:10);  % 10^-15 to 10^0

    % Create a figure for the plot
    figure;
    hold on;

    % % Dummy handles for the legend
    h1 = plot(NaN, NaN, 'bx', 'MarkerSize', 8, 'LineWidth', 1.5);
    h2 = plot(NaN, NaN, 'rx', 'MarkerSize', 5, 'LineWidth', 1);

    % Loop through each combination of piScalar and plScalar
    for i = 1:length(piRange)
        for j = 1:length(plRange)
            piScalar = piRange(i);
            plScalar = plRange(j);

            % Define piVals and plVals
            piVals = piScalar * ones(1, numOfDGs);
            plVals = plScalar * ones(1, numOfLines);

            % Call the centralizedLocalControlDesign function
            [~, ~, statusLocalController] = centralizedLocalControlDesign(DG, Line, B_il, BarGamma, piVals, plVals);

            % Plot the result with a cross marker
            if statusLocalController == 1
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
    title('Local Controller Status Across piScalar and plScalar');
    grid on;
    legend([h1, h2], 'Status 1', 'Status 0');  % Use the dummy handles for the legend
    hold off;
end
% %% Second Method
% function plotLocalControlDesign(DG, Line, B_il, BarGamma, numOfDGs, numOfLines)
%     % Turn off all warnings
%     warning off;
% 
%     % Define the correct range for piScalar and plScalar
%     piRange = 10.^(-5:.5:0);  % Adjust the range as needed
%     plRange = 10.^(-10:.5:0);  % Adjust the range as needed
% 
%     % Create a figure for the plot
%     figure;
%     hold on;
% 
%     % Dummy handles for the legend
%     h1 = plot(NaN, NaN, 'bx', 'MarkerSize', 8, 'LineWidth', 1.5);  % Blue cross for both statuses 1
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
%             % Plot a cross based on the combined status
%             if statusDG == 1 && statusLine == 1
%                 % Blue cross if both statuses are 1
%                 plot(log10(piScalar), log10(plScalar), 'bx', 'MarkerSize', 8, 'LineWidth', 1.5);
%             else
%                 % Red cross if any status is 0
%                 plot(log10(piScalar), log10(plScalar), 'rx', 'MarkerSize', 8, 'LineWidth', 1.5);
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
