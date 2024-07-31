function DG = GenerateDG(R0,L0,C0,Load0,Y0)

    Ri = R0 + 0.01*rand()
    Li = L0 + 0.01*rand();
    Ci = C0 + 0.001*rand();
    Yi = Y0 + 0.1*rand();
    Loadi = 1/ Yi;

    A_DG{dg} = [-Y_values(dg)/C_values(dg), 1/C_values(dg), 0;
                -1/L_values(dg), -R_values(dg)/L_values(dg), 0;
                1, 0, 0];
    B_DG{dg} = [0; 1/L_values(dg); 0];
    
    DG.R = Ri;
    DG.L = Li;
    DG.C = Ci;
    DG.Y = Yi;
end

