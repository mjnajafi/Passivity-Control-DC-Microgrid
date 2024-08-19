% This function, GenerateLine, generates a transmission line with 
% randomly perturbed electrical parameters based on initial values provided 
% as inputs (Rl0, Ll0). The function creates and stores the line's attributes, 
% including the resistance (R), inductance (L), and state-space matrices (A and B)
% in a structure named Line.

function Line = GenerateLine(Rl0,Ll0)

    % Generating
    Rl = Rl0 + 0.01*rand();
    Ll = Ll0 + 0.01*rand();

    Al = -Rl / Ll;
    Bl = 1 / Ll;

    % Storing
    Line.R = Rl;
    Line.L = Ll;
    Line.A = Al;
    Line.B = Bl;
end

