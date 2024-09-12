% This function, GenerateLine, generates a transmission line with
% more significant variations in electrical parameters based on initial 
% values provided as inputs (Rl0, Ll0). The function creates and stores the line's 
% attributes, including resistance (R), inductance (L), and state-space 
% matrices (A and B) in a structure named Line.

function Line = GenerateLine(Rl0, Ll0)

    % Introduce more significant variations in line parameters
    % For example, vary parameters within a larger range
    variationFactorRl = 0.4;  % 40% variation in line resistance
    variationFactorLl = 0.3;  % 30% variation in line inductance

    % Generate new values for the line parameters with larger variations
    Rl = Rl0 * (1 + variationFactorRl * (2*rand() - 1));  
    Ll = Ll0 * (1 + variationFactorLl * (2*rand() - 1));  

    % State-space matrices A and B for the transmission line
    Al = -Rl / Ll;
    Bl = 1 / Ll;

    % Storing the generated values in the Line structure
    Line.R = Rl;
    Line.L = Ll;
    Line.A = Al;
    Line.B = Bl;
end

% 
% function Line = GenerateLine(Rl0,Ll0)
% 
%     % Generating
%     Rl = Rl0 + 0.01*rand();
%     Ll = Ll0 + 0.01*rand();
% 
%     Al = -Rl / Ll;
%     Bl = 1 / Ll;
% 
%     % Storing
%     Line.R = Rl;
%     Line.L = Ll;
%     Line.A = Al;
%     Line.B = Bl;
% end

