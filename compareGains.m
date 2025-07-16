function compareGains()
    % Load both result files
    hardData = load('hard_constraint_gains.mat');
    softData = load('soft_constraint_gains.mat');
    
    hardMetrics = hardData.metrics;
    softMetrics = softData.metrics;
    
    % Calculate ratios
    maxGainRatio = hardMetrics.maxGain / softMetrics.maxGain;
    avgGainRatio = hardMetrics.avgGain / softMetrics.avgGain;
    sumMagnitudeRatio = hardMetrics.sumMagnitude / softMetrics.sumMagnitude;
    frobeniusNormRatio = hardMetrics.frobeniusNorm / softMetrics.frobeniusNorm;
    connectionRatio = hardMetrics.numConnections / softMetrics.numConnections;
    nonzeroRatio = hardMetrics.totalNonzero / softMetrics.totalNonzero;
    avgPerConnectionRatio = hardMetrics.avgGainPerConnection / softMetrics.avgGainPerConnection;
    
    % Display comparison
    fprintf('\n=== Comparison: Hard vs Soft Constraints ===\n');
    fprintf('                      Hard      Soft      Ratio (Hard/Soft)\n');
    fprintf('Max Gain:            %.2f     %.2f     %.4f\n', hardMetrics.maxGain, softMetrics.maxGain, maxGainRatio);
    fprintf('Avg Gain (non-zero): %.2f     %.2f     %.4f\n', hardMetrics.avgGain, softMetrics.avgGain, avgGainRatio);
    fprintf('Sum of Magnitudes:   %.2f     %.2f     %.4f\n', hardMetrics.sumMagnitude, softMetrics.sumMagnitude, sumMagnitudeRatio);
    fprintf('Frobenius Norm:      %.2f     %.2f     %.4f\n', hardMetrics.frobeniusNorm, softMetrics.frobeniusNorm, frobeniusNormRatio);
    fprintf('Num of Connections:  %d        %d        %.4f\n', hardMetrics.numConnections, softMetrics.numConnections, connectionRatio);
    fprintf('Total Non-zero:      %d        %d        %.4f\n', hardMetrics.totalNonzero, softMetrics.totalNonzero, nonzeroRatio);
    fprintf('Avg Gain Per Link:   %.2f     %.2f     %.4f\n', hardMetrics.avgGainPerConnection, softMetrics.avgGainPerConnection, avgPerConnectionRatio);
    
    % Plot comparison
    figure;
    metrics_to_plot = [
        hardMetrics.maxGain, softMetrics.maxGain; 
        hardMetrics.avgGain, softMetrics.avgGain; 
        % hardMetrics.sumMagnitude/1000, softMetrics.sumMagnitude/1000;
        % hardMetrics.frobeniusNorm/1000, softMetrics.frobeniusNorm/1000;
    ];
    
    bar(metrics_to_plot);
    set(gca, 'XTickLabel', {'Max Gain', 'Avg Gain', 'Sum Mag/1000', 'Frob Norm/1000'});
    legend('Hard Constraints', 'Soft Constraints');
    title('Comparison of Controller Gain Metrics');
    ylabel('Magnitude');
    grid on;
    
    % Create a separate figure for normalized comparison (where values are relative percentages)
    figure;
    normalized_metrics = [
        1, softMetrics.maxGain/hardMetrics.maxGain;
        1, softMetrics.avgGain/hardMetrics.avgGain;
        % 1, softMetrics.sumMagnitude/hardMetrics.sumMagnitude;
        % 1, softMetrics.frobeniusNorm/hardMetrics.frobeniusNorm;
    ];
    
    bar(normalized_metrics);
    set(gca, 'XTickLabel', {'Max Gain', 'Avg Gain', 'Sum Magnitude', 'Frobenius Norm'});
    legend('Hard (normalized)', 'Soft (relative to Hard)');
    title('Normalized Comparison of Controller Gain Metrics');
    ylabel('Relative Magnitude (Hard = 1)');
    grid on;
end