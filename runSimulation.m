% Filename: runSimulation.m
% Define parameters
params = struct('C_ti', 0.001, 'L_ti', 0.01, 'R_ti', 0.05, ...
                'Y_Li', 0.02, 'P_Li', 0.03, 'u_i0', 1, ...
                'Bar_I_Li', 0.1, 'V_ri', 0.5);

% Define input values (replace with actual data)
u = struct('B_il', B_il, 'u_ij', u_ij, 'I_l', I_l);

% Initial conditions
x0 = [initial_V_i; initial_I_ti; initial_v_i]; % Replace with actual values

% Time span for the simulation
tspan = [0, T_final]; % Replace with actual final time

% Solve the ODE
[t, X] = ode45(@(t, x) stateSpaceWithController(t, x, u), tspan, x0);

% Plot results or perform further analysis
plot(t, X);
xlabel('Time');
ylabel('States');
legend('V_i', 'I_ti', 'v_i');
title('System Response');
