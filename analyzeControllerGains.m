function analyzeControllerGains(K, isSoft)
    % Function to analyze controller gains
    % Inputs:
    %   K - Cell array of gain matrices
    %   isSoft - Flag indicating soft (1) or hard (0) constraints
    
    % Generate filename based on constraint type
    if isSoft == 1
        filename = 'soft_constraint_gains.mat';
        constraintType = 'Soft';
    else
        filename = 'hard_constraint_gains.mat';
        constraintType = 'Hard';
    end
    
    % Calculate metrics
    [num_rows, num_cols] = size(K);
    maxGainValue = 0;
    totalMagnitude = 0;
    numConnections = 0;
    totalNonzero = 0;
    frobeniusNorm = 0;
    
    % Process each block
    fprintf('\n=== %s Constraint Controller Gain Analysis ===\n', constraintType);
    for i = 1:num_rows
        for j = 1:num_cols
            if i ~= j  % Skip self-connections
                K_ij = K{i,j};
                nonzero = nnz(K_ij);
                maxVal = max(max(abs(K_ij)));
                sumMag = sum(sum(abs(K_ij)));
                frobNorm = norm(K_ij, 'fro');
                
                if nonzero > 0
                    fprintf('Connection DG%d→DG%d: Max gain = %.4f, Sum = %.4f\n', j, i, maxVal, sumMag);
                    numConnections = numConnections + 1;
                end
                
                maxGainValue = max(maxGainValue, maxVal);
                totalMagnitude = totalMagnitude + sumMag;
                frobeniusNorm = frobeniusNorm + frobNorm^2;
                totalNonzero = totalNonzero + nonzero;
            end
        end
    end
    
    frobeniusNorm = sqrt(frobeniusNorm);
    
    % Calculate average metrics
    if totalNonzero > 0
        avgGain = totalMagnitude / totalNonzero;
    else
        avgGain = 0;
    end
    
    if numConnections > 0
        avgGainPerConnection = totalMagnitude / numConnections;
    else
        avgGainPerConnection = 0;
    end
    
    % Display summary
    fprintf('\n--- Summary for %s Constraints ---\n', constraintType);
    fprintf('Maximum gain: %.4f\n', maxGainValue);
    fprintf('Average gain (non-zero): %.4f\n', avgGain);
    fprintf('Sum of all gain magnitudes: %.4f\n', totalMagnitude);
    fprintf('Frobenius norm: %.4f\n', frobeniusNorm);
    fprintf('Number of connections: %d\n', numConnections);
    fprintf('Total non-zero elements: %d\n', totalNonzero);
    fprintf('Average gain per connection: %.4f\n', avgGainPerConnection);
    
    % Save results
    metrics = struct();
    metrics.K = K;
    metrics.maxGain = maxGainValue;
    metrics.avgGain = avgGain;
    metrics.sumMagnitude = totalMagnitude;
    metrics.frobeniusNorm = frobeniusNorm;
    metrics.numConnections = numConnections;
    metrics.totalNonzero = totalNonzero;
    metrics.avgGainPerConnection = avgGainPerConnection;
    
    save(filename, 'metrics');
    fprintf('Results saved to %s\n', filename);
    
    % If both files exist, compare them
    otherFilename = '';
    if isSoft == 1
        otherFilename = 'hard_constraint_gains.mat';
    else
        otherFilename = 'soft_constraint_gains.mat';
    end
    
    if exist(otherFilename, 'file')
        fprintf('\nBoth constraint types have been analyzed. Comparing them...\n');
        compareGains();
    else
        fprintf('\nRun the simulation with isSoft = %d to compare both constraint types.\n', ~isSoft);
    end
end