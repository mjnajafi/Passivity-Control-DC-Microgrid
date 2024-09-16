function [commMatrix] = generateCommunicationTopology(coords, A_ij, K)
    % Function to add communication links to an existing physical topology


    epsilon = 10e-6;
    A = A_ij; % Adjacency matrix of the DG-DG communication topology
    % Number of Distributed Generators (DGs)
    numOfDGs = size(coords, 1);

    % Plot the existing physical topology
    figure; hold on;
    for i = 1:numOfDGs
        for j = i+1:numOfDGs
            if A(i, j) == 1
                % Coordinates of the physical link
                x = [coords(i,1) coords(j,1)];
                y = [coords(i,2) coords(j,2)];
                % Plot solid blue line for physical link
                plot(x, y, 'k-', 'LineWidth', 2);
            end
        end
    end

    % Plot DGs as black circles
    scatter(coords(:,1), coords(:,2), 100, 'bo', 'filled');
    title('Physical and Communication Topology');
    xlabel('X-coordinate');
    ylabel('Y-coordinate');
    grid on;
    
    % Initialize communication adjacency matrix
    commMatrix = zeros(numOfDGs, numOfDGs);

    % Add communication links based on controller gains (K matrix)
    for i = 1:numOfDGs
        for j = i+1:numOfDGs
            % Check if the norm of the controller gain is greater than epsilon
            normVal = max(max(abs(K{i, j})));
            if normVal > epsilon
                % Update communication adjacency matrix
                commMatrix(i, j) = 1;
                commMatrix(j, i) = 1;

                % Plot dashed red line for communication link
                x = [coords(i,1) coords(j,1)];
                y = [coords(i,2) coords(j,2)];
                plot(x, y, 'r--', 'LineWidth', 1.5);
            end
        end
    end

    hold off;
end
