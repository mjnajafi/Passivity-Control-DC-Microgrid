% This function, generateMicrogridTopology, generates and visualizes a random microgrid 
% topology for a given number of Distributed Generators (DGs). The topology is defined 
% by randomly placed DGs within a 2D space, where connections between DGs are determined 
% based on a distance threshold. The function outputs the coordinates of the DGs, 
% the adjacency matrix representing connections between DGs, and the B_il matrix 
% that describes the incidence relationship between DGs and the transmission lines.

function [coords, adjMatrix, B_il] = generateMicrogridTopology(numOfDGs, threshold)
    % Generate random points and visualize the microgrid topology.
    %
    % Inputs:
    %   numOfDGs - Number of Distributed Generators (DGs).
    %   threshold - Distance threshold to determine if DGs are connected.
    %
    % Outputs:
    %   coords - Coordinates of the DGs.
    %   adjMatrix - Adjacency matrix of the DGs.
    %   B_il - Matrix representing connections between DGs and lines.

    % Step 1: Generate random points for DGs
    coords = rand(numOfDGs, 2); % Generate random coordinates in [0, 1]^2

    % Step 2: Initialize adjacency matrix
    adjMatrix = zeros(numOfDGs, numOfDGs);

    % Create a list to store lines (edges) and their connections
    lines = [];
    lineIndex = 1;

    % Determine connections based on distance threshold
    for i = 1:numOfDGs
        for j = i+1:numOfDGs
            distance = norm(coords(i,:) - coords(j,:));
            if distance < threshold
                % Create a line connecting DG i and DG j
                lines = [lines; i, j];
                % Update adjacency matrix
                adjMatrix(i, j) = 1;
                adjMatrix(j, i) = 1;
            end
        end
    end

    % Step 3: Generate B_il matrix
    % Number of lines
    numOfLines = size(lines, 1);
    B_il = zeros(numOfDGs, numOfLines);

    % Fill the B_il matrix
    for l = 1:numOfLines
        i = lines(l, 1);
        j = lines(l, 2);
        B_il(i, l) = 1; % DG i is an in-neighbor of line k
        B_il(j, l) = -1;  % DG l is an out-neighbor of line k
        
    end

    % Step 4: Plot the graph
    figure;
    hold on;

    % Plot DGs as points
    scatter(coords(:,1), coords(:,2), 100, 'filled', 'b');

    % Annotate nodes
    for i = 1:numOfDGs
        text(coords(i,1), coords(i,2), num2str(i), 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right');
    end

    % Plot lines with rectangle markers
    for k = 1:numOfLines
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


        % Annotate the line number
        text(midX, midY, num2str(k), 'Color', 'k', 'FontWeight', 'bold', ...
             'VerticalAlignment', 'middle', 'HorizontalAlignment', 'center');
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

    %% Draw communication topology on top of the figure

end



% function outputArg = drawTopology(obj,figNum)
%     figure(figNum); hold on;
% 
%     if ~isempty(obj.graphics1)
%         delete(obj.graphics1);
%         delete(obj.graphics2);
%     end
% 
%     numOfLinks = length(obj.topology.startNodes);
%     for i = 1:1:numOfLinks
%         % Draw a link
%         startVehicleIndex = obj.topology.startNodes(i);
%         endVehicleIndex = obj.topology.endNodes(i);
% 
%         startPos = obj.vehicles(startVehicleIndex).states(1) - obj.vehicles(startVehicleIndex).vehicleParameters(2)/2;
%         endPos = obj.vehicles(endVehicleIndex).states(1) - obj.vehicles(endVehicleIndex).vehicleParameters(2)/2;
%         midPos = (startPos + endPos)/2;
% 
%         % midPointHeight = -5*sign(startPos-endPos)+0.05*abs(startPos-endPos)-0.5*(startPos<endPos); % 4
%         % Simplify mid-point height calculation
%         % Define baseline deviation ranges
%         minBaselineDeviation = 1; % Minimum deviation from the baseline
%         maxBaselineDeviation = 5; % Maximum deviation from the baseline
%         % Calculate mid-point height deviation
%         directionSign = sign(startPos - endPos);
%         distance = abs(startPos - endPos);
% 
%         % Scale the deviation within the specified range
%         deviationRange = maxBaselineDeviation - minBaselineDeviation;
%         scaledDeviation = (deviationRange / 100) * distance + minBaselineDeviation;
% 
%          % Adjust mid-point height based on direction and vehicle height
%         vehicleHeightCorrection = 1.5 * (startPos > endPos) * directionSign;
%         midPointHeight = -directionSign * scaledDeviation + vehicleHeightCorrection;
% 
% 
%         startPosY = obj.vehicles(startVehicleIndex).vehicleParameters(2)*3/8;
%         endPosY = obj.vehicles(endVehicleIndex).vehicleParameters(2)*3/8;
%         %obj.graphics(i) = plot([startPos,midPos,endPos],[startPosY,midPointHeight,endPosY],'-b');
% 
%         % Plotting the Spline
%         x = [startPos,midPos,endPos];
%         y = [startPosY,midPointHeight,endPosY];
%         stepSize = (endPos-startPos)/20; 
%         xx = startPos:stepSize:endPos;
%         yy = spline(x,y,xx);
%         obj.graphics1(i) = plot(xx,yy,'-b');
% 
%         % Plotting the arrowhead (polyshape)
%         polyPosX = midPos;
%         polyPosY = midPointHeight;
%         polySize = 0.3;
%         polyVertX = [-0.5, 1, -0.5];
%         polyVertY = [0.5, 0, -0.5];
%         if startPos > endPos
%             polyVertX = -polyVertX;
%         end
%         arrowHead = polyshape(polyPosX + polySize * polyVertX, polyPosY + polySize * polyVertY);
%         obj.graphics2(i) = plot(arrowHead, 'EdgeColor', 'k', 'FaceColor', 'b');
%     end            
% end