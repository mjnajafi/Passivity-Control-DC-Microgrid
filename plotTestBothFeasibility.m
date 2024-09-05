function plotTestBothFeasibility(DG, Line, B_il, BarGamma, A_ij, isSoft, numOfDGs, numOfLines)
    % Turn off all warnings
    warning off;

    step_size = 0.5;

    % Define the correct range for piScalar and plScalar
    piRange = 10.^(-5:step_size:5);  % Adjust the range as needed
    plRange = 10.^(-5:step_size:5);  % Adjust the range as needed

    % Set the square size (length of each side of the square)
    squareSize = step_size;  % Adjust this size as needed

    % Number of random points to generate around each main point
    numRandomPoints = 10;

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

            try
                % Call the globalControlDesign function
                [DG, Line, statusGlobalController, gammaTildeVal, K, C, BarC, H, P_iVal, P_lVal] = globalControlDesign(DG, Line, A_ij, B_il, BarGamma, isSoft);

                % Calculate the center point in log10 scale
                logPiScalar = log10(piScalar);
                logPlScalar = log10(plScalar);

                % Plot the result based on the combined status
                if statusLocalController == 1
                    % Blue cross for local controller status 1
                    plot(logPiScalar, logPlScalar, 'bx', 'MarkerSize', 8, 'LineWidth', 1.5);
                else
                    % Red cross if local controller status is 0
                    plot(logPiScalar, logPlScalar, 'rx', 'MarkerSize', 8, 'LineWidth', 1.5);
                end

                if statusGlobalController == 1
                    % Blue circle for global controller status 1
                    plot(logPiScalar, logPlScalar, 'bo', 'MarkerSize', 8, 'LineWidth', 1.5);
                else
                    % Red circle if global controller status is 0
                    plot(logPiScalar, logPlScalar, 'ro', 'MarkerSize', 8, 'LineWidth', 1.5);
                end

                % Draw the square around the point
                halfSize = squareSize / 2;
                xMin = logPiScalar - halfSize;
                xMax = logPiScalar + halfSize;
                yMin = logPlScalar - halfSize;
                yMax = logPlScalar + halfSize;

                % Draw horizontal lines (top and bottom)
                plot([xMin, xMax], [yMax, yMax], 'k--', 'LineWidth', 1);  % Top line
                plot([xMin, xMax], [yMin, yMin], 'k--', 'LineWidth', 1);  % Bottom line

                % Draw vertical lines (left and right)
                plot([xMin, xMin], [yMin, yMax], 'k--', 'LineWidth', 1);  % Left line
                plot([xMax, xMax], [yMin, yMax], 'k--', 'LineWidth', 1);  % Right line

                % Generate random points within the square
                for k = 1:numRandomPoints
                    % Generate random piVals and plVals within the square area
                    randomLogPi = logPiScalar + (rand() - 0.5) * squareSize;
                    randomLogPl = logPlScalar + (rand() - 0.5) * squareSize;

                    randomPiScalar = 10^randomLogPi;
                    randomPlScalar = 10^randomLogPl;

                    % Define random piVals and plVals
                    randomPiVals = randomPiScalar * ones(1, numOfDGs);
                    randomPlVals = randomPlScalar * ones(1, numOfLines);

                    % Evaluate feasibility for the random points
                    [DG, Line, randomStatusLocalController] = centralizedLocalControlDesign(DG0, Line0, B_il, BarGamma, randomPiVals, randomPlVals);

                    try
                        [DG, Line, randomStatusGlobalController, ~, ~, ~, ~, ~, ~] = globalControlDesign(DG, Line, A_ij, B_il, BarGamma, isSoft);

                        % Plot the result for the random points
                        if randomStatusLocalController == 1
                            % Blue cross for local controller status 1
                            plot(randomLogPi, randomLogPl, 'bx', 'MarkerSize', 7, 'LineWidth', 1.5);
                        else
                            % Red cross if local controller status is 0
                            plot(randomLogPi, randomLogPl, 'rx', 'MarkerSize', 7, 'LineWidth', 1.5);
                        end

                        if randomStatusGlobalController == 1
                            % Blue circle for global controller status 1
                            plot(randomLogPi, randomLogPl, 'bo', 'MarkerSize', 7, 'LineWidth', 1.5);
                        else
                            % Red circle if global controller status is 0
                            plot(randomLogPi, randomLogPl, 'ro', 'MarkerSize', 7, 'LineWidth', 1.5);
                        end
                    catch ME
                        % If an error occurs with the random point, skip it
                        continue;
                    end
                end

            catch ME
                % If an error occurs, skip to the next iteration
                continue;
            end
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
