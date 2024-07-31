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

