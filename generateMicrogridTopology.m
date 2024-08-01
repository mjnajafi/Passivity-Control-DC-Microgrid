function [coords, adjMatrix, B_il] = generateMicrogridTopology(numberOfDGs, threshold)
    % Generate random points and visualize the microgrid topology.
    %
    % Inputs:
    %   numberOfDGs - Number of Distributed Generators (DGs).
    %   threshold - Distance threshold to determine if DGs are connected.
    %
    % Outputs:
    %   coords - Coordinates of the DGs.
    %   adjMatrix - Adjacency matrix of the DGs.
    %   B_il - Matrix representing connections between DGs and lines.

    % Step 1: Generate random points for DGs
    coords = rand(numberOfDGs, 2); % Generate random coordinates in [0, 1]^2

    % Step 2: Initialize adjacency matrix
    adjMatrix = zeros(numberOfDGs, numberOfDGs);

    % Create a list to store lines (edges) and their connections
    lines = [];
    lineIndex = 1;

    % Determine connections based on distance threshold
    for i = 1:numberOfDGs
        for j = i+1:numberOfDGs
            distance = norm(coords(i,:) - coords(j,:));
            if distance < threshold
                % Create a line connecting DG i and DG j
                lines = [lines; i, j];
                % Update adjacency matrix
                adjMatrix(i, j) = 1;
                adjMatrix(j, i) = -1;
            end
        end
    end

    % Step 3: Generate B_il matrix
    % Number of lines
    numberOfLines = size(lines, 1);
    B_il = zeros(numberOfDGs, numberOfLines);

    % Fill the B_il matrix
    for k = 1:numberOfLines
        i = lines(k, 1);
        l = lines(k, 2);
        B_il(i, k) = 1; % DG i is an in-neighbor of line k
        B_il(l, k) = -1; % DG l is an out-neighbor of line k
    end

    % Step 4: Plot the graph
    figure;
    hold on;

    % Plot DGs as points
    scatter(coords(:,1), coords(:,2), 100, 'filled', 'b');

    % Annotate nodes
    for i = 1:numberOfDGs
        text(coords(i,1), coords(i,2), num2str(i), 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right');
    end

    % Plot lines with rectangle markers
    for k = 1:numberOfLines
        i = lines(k, 1);
        l = lines(k, 2);
        % Coordinates of the line
        x = [coords(i,1) coords(l,1)];
        y = [coords(i,2) coords(l,2)];
        % Midpoint of the line
        midX = (coords(i,1) + coords(l,1)) / 2;
        midY = (coords(i,2) + coords(l,2)) / 2;
        
        % Plot line
        plot(x, y, 'k-', 'LineWidth', 1.5);
        
        % Plot rectangle marker at the midpoint
        rectangle('Position', [midX - 0.01, midY - 0.01, 0.02, 0.02], ...
                  'FaceColor', 'r', 'EdgeColor', 'k');
    end

    % Set plot properties
    title('Microgrid Topology');
    xlabel('X Coordinate');
    ylabel('Y Coordinate');
    axis equal;
    grid on;
    hold off;

    % Display results
    disp('Coordinates of DGs:');
    disp(coords);
    disp('Adjacency Matrix:');
    disp(adjMatrix);
    disp('B_il Matrix:');
    disp(B_il);
end
