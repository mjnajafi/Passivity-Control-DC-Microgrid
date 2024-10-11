% This function, GenerateDG, generates a Distributed Generator (DG) with
% more significant variations in electrical parameters based on initial values 
% provided as inputs (R0, L0, C0, RL0). The function creates and stores the 
% system's attributes, including resistance (R), inductance (L), capacitance (C), 
% load admittance (Y), state-space matrices (A and B), and coordinates (coords_i) 
% in a structure named DG.

function DG = GenerateDG(R0, L0, C0, RL0, IL0, coords_i)

    % Introduce more significant variations in DG parameters
    % For example, vary parameters within a larger range
    variationFactorR = 0.9;   % 50% variation in R
    variationFactorL = 0.9;   % 20% variation in L
    variationFactorC = 0.9;   % 30% variation in C
    variationFactorRL = 0.9;  % 40% variation in RL
    variationFactorIL = 0.9;  % 30% variation in IL

    % Generate new values for the parameters with larger variations
    Ri = R0 * (1 + variationFactorR * (2*rand() - 1));   
    Li = L0 * (1 + variationFactorL * (2*rand() - 1));   
    Ci = C0 * (1 + variationFactorC * (2*rand() - 1));   
    RLi = RL0 * (1 + variationFactorRL * (2*rand() - 1)); 
    Yi = 1 / RLi;    
    ILi = IL0 * (1 + variationFactorIL * (2*rand() - 1)); 

    % State-space matrices A and B
    Ai = [-Yi/Ci, 1/Ci, 0;
          -1/Li, -Ri/Li, 0;
           1, 0, 0];

    Bi = [0; 1/Li; 0];

    % Storing the generated values in the DG structure
    DG.R = Ri;
    DG.L = Li;
    DG.C = Ci;
    DG.Y = Yi;
    DG.IL = ILi;
    DG.A = Ai;
    DG.B = Bi;
    DG.coordinates = coords_i;
end


% 
% function DG = GenerateDG(R0,L0,C0,RL0,IL0,coords_i)
% 
%     % Generating
%     Ri = R0 + 0.01*rand();
%     Li = L0 + 0.01*rand();
%     Ci = C0 + 0.001*rand();
%     RLi = RL0 + 0.01*rand();
%     Yi = 1 / RLi;
%     ILi = IL0 + 0.1*rand();
% 
% 
% 
% 
%     Ai = [-Yi/Ci, 1/Ci, 0;
%                -1/Li, -Ri/Li, 0;
%                 1, 0, 0];
% 
% 
% 
% 
% 
%     Bi = [0; 1/Li; 0];
% 
% 
%     % Storing
%     DG.R = Ri;
%     DG.L = Li;
%     DG.C = Ci;
%     DG.Y = Yi;
%     DG.IL = ILi;
%     DG.A = Ai;
%     DG.B = Bi;
%     DG.coordinates = coords_i;
% end
% 
