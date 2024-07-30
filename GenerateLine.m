function Line = GenerateLine(Rl0,Ll0)

    % Generating
    Rl = Rl0 + 0.01*rand();
    Ll = Ll0 + 0.01*rand();

    % Storing
    Line.R = Rl;
    Line.L = Ll;
    Line.A = [Rl, Ll];
    Line.B = [Rl]
end

