function DG = GenerateDG(R0, L0, C0, RL0, IL0, Pn0, coords_i)
    % Input voltage and reference specifications
    DG.sourceVoltage = 120;    % Input DC voltage source (V)
    DG.refVoltage = 48;        % Reference output voltage (V)
    
    % Switching frequency (from PWM settings)
    DG.switchingFreq = 12500;  % 12.5 kHz
    
    % Parameter variations
    variationFactorR = 0.2;    % 20% variation in R
    variationFactorL = 0.2;    % 20% variation in L
    variationFactorC = 0.2;    % 20% variation in C
    variationFactorRL = 0.2;   % 20% variation in RL
    variationFactorIL = 0.2;   % 20% variation in IL
    variationFactorPn = 0.2;

    % Generate parameters with variations
    Pni = Pn0 * (1 + variationFactorPn * (2*rand() - 1));
    Ri = R0 * (1 + variationFactorR * (2*rand() - 1));
    Li = L0 * (1 + variationFactorL * (2*rand() - 1));
    Ci = C0 * (1 + variationFactorC * (2*rand() - 1));
    RLi = RL0 * (1 + variationFactorRL * (2*rand() - 1));
    Yi = 1 / RLi;
    ILi = IL0 * (1 + variationFactorIL * (2*rand() - 1));
    
    
    % Calculate current ripple
    deltaI = DG.sourceVoltage/(4*Li*DG.switchingFreq);
    
    % Calculate duty cycle
    duty_cycle_nominal = DG.refVoltage/DG.sourceVoltage;
    
    % Calculate currents
    I_RL = DG.refVoltage / RLi;        % Current through resistive load
    I_IL = ILi;                        % Constant current load
    DG.ratedCurrent = I_RL + I_IL;     % Total current capability
    
    % Calculate power losses
    I_total = DG.ratedCurrent;
    P_losses_R = I_total^2 * Ri;                           % Internal resistance losses
    P_losses_switching = 0.05 * (DG.refVoltage * I_total); % Switching losses (5%)
    
    % Calculate total power rating
    P_load = DG.refVoltage * DG.ratedCurrent;             % Power delivered to load
    DG.powerRating = P_load + P_losses_R + P_losses_switching;
    
    % Add 20% margin for safety
    DG.powerRating = DG.powerRating * 1.2;
    
    % Round to nearest 100W for practical values
    DG.powerRating = ceil(DG.powerRating/100) * 100;
    
    % Store power rating for current sharing
    DG.Pn = DG.powerRating;
    
    % Calculate resonant frequency
    DG.resonantFreq = 1/(2*pi*sqrt(Li*Ci));
    
    % Store calculations
    DG.currentRipple = deltaI;
    DG.dutyNominal = duty_cycle_nominal;
    
    % State-space matrices
    Ai = [-Yi/Ci, 1/Ci, 0;
          -1/Li, -Ri/Li, 0;
          1, 0, 0];
    Bi = [0; 1/Li; 0];
    
    % Store all parameters
    DG.R = Ri;
    DG.RL = RLi;
    DG.L = Li;
    DG.C = Ci;
    DG.Y = Yi;
    DG.IL = ILi;
    DG.A = Ai;
    DG.B = Bi;
    DG.coordinates = coords_i;
    DG.Pn = Pni;
    
    % Print specifications
    fprintf('\nDG Specifications:\n');
    fprintf('Source Voltage: %.2f V\n', DG.sourceVoltage);
    fprintf('Reference Voltage: %.2f V\n', DG.refVoltage);
    fprintf('Rated Current: %.2f A\n', DG.ratedCurrent);
    fprintf('Power Rating: %.2f W\n', DG.powerRating);
    fprintf('Power Rating (Pn): %.2f W\n', DG.Pn);
    fprintf('Current through Resistive Load: %.2f A\n', I_RL);
    fprintf('Constant Current Load: %.2f A\n', I_IL);
    fprintf('Power Losses (Resistance): %.2f W\n', P_losses_R);
    fprintf('Power Losses (Switching): %.2f W\n', P_losses_switching);
    fprintf('Current Ripple: %.2f A (%.1f%%)\n', DG.currentRipple, ...
        (DG.currentRipple/DG.ratedCurrent)*100);
    fprintf('Component Values:\n');
    fprintf(' Inductance: %.3f H\n', DG.L);
    fprintf(' Capacitance: %.3f F\n', DG.C);
    fprintf(' Resistance: %.3f Ω\n', DG.R);
    fprintf(' Load Resistance: %.3f Ω\n', 1/DG.Y);
end