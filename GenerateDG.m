% This function, GenerateDG, generates a Distributed Generator (DG) with
% randomly perturbed electrical parameters based on initial values provided 
% as inputs (R0, L0, C0, RL0). The function creates and stores the system's 
% attributes, including the resistance (R), inductance (L), capacitance (C), 
% load admittance (Y), state-space matrices (A and B), and coordinates 
% (coords_i) in a structure named DG.

function DG = GenerateDG(R0,L0,C0,RL0,IL0,coords_i)

    % Generating
    Ri = R0 + 0.01*rand();
    Li = L0 + 0.01*rand();
    Ci = C0 + 0.001*rand();
    RLi = RL0 + 0.01*rand();
    Yi = 1 / RLi;
    ILi = IL0 + 0.1*rand();
    
    Ai = [-Yi/Ci, 1/Ci, 0;
               -1/Li, -Ri/Li, 0;
                1, 0, 0];


   
    
    
    Bi = [0; 1/Li; 0];

       
    % Storing
    DG.R = Ri;
    DG.L = Li;
    DG.C = Ci;
    DG.Y = Yi;
    DG.IL = ILi;
    DG.A = Ai;
    DG.B = Bi;
    DG.coordinates = coords_i;
end

