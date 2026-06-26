
% Mass density of the ferromagnetic core (used for gravimetric conversion)

rho = 8500;   % [kg/m^3]  

% Magnetic stimulus configuration for hysteresis loop characterization

Hmax_kOe = 7;    % Maximum applied magnetic field [kOe]
N = 12000; % Temporal resolution (number of sampling points)

% Applied field conversion to International System of Units (SI)
Hmax_Am = Hmax_kOe * 79577;  % Conversion factor: 1 kOe = 79577 A/m

H_Am = [linspace(-Hmax_Am, Hmax_Am, N/2), linspace(Hmax_Am, -Hmax_Am, N/2)];

% Memory preallocation for state vectors
M_Am = zeros(size(H_Am));
B_T  = zeros(size(H_Am));

% Open-loop quasi-static simulation execution
for i = 1:numel(H_Am)
    [B_T(i), M_Am(i)] = Jiles_Antherton_myself(H_Am(i));
end

% Variable conversion to CGS / literature-standard units

H_kOe   = H_Am / 79577; % Applied magnetic field [kOe]
M_emu_g = (M_Am / rho); % Specific/gravimetric magnetization [emu/g]

% Graphical representation of the M(H) magnetization curve
figure;
plot(H_kOe, M_emu_g, 'LineWidth', 1.4);
grid on;
xlabel('Magnetic Field (kOe)');
ylabel('Magnetization (emu/g)');
title('M(H) - Jiles-Atherton (kOe e emu/g)');