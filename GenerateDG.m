function DG = GenerateDG(R0,L0,C0,RL0)

    % Generating
    Ri = R0 + 0.01*rand();
    Li = L0 + 0.01*rand();
    Ci = C0 + 0.001*rand();
    RLi = RL0 + 0.01*rand();
    Yi = 1 / RLi;
    
    Ai = [-Yi/Ci, 1/Ci, 0;
               -1/Li, -Ri/Li, 0;
                1, 0, 0];
    
    Bi = [0; 1/Li; 0];
    
    % Storing
    DG.R = Ri;
    DG.L = Li;
    DG.C = Ci;
    DG.Y = Yi;
    DG.A = Ai;
    DG.B = Bi;
end

