function [B,M]= Jiles_Antherton(H)

%Jiles-Atherton model for ferromagnetic materials
% Allocation of persistent variables for magnetic state retention

persistent H_prev delta_prev M_state

% State initialization at the initial time step (t = 0)
if isempty(M_state)
    
    H_prev = 0;
    delta_prev = 1;
    M_state = 0;

end

% Universal physical constants

mu0=4*pi*10^-7;% Magnetic permeability of vacuum [H/m]

% Phenomenological material parameters (Fe50Ni50)

M_sat =1.4e6; % Saturation magnetization [A/m]
a = 7.4e4;     % Domain wall density parameter [A/m] 
k = 7000;     % Pinning energy density / Coercivity parameter [A/m]
c = 4e-4;     % Magnetization reversibility
alpha = 6e-6;  % Mean-field coupling parameter (inter-domain interaction)


% Definition of the value delta (either -1 or +1), depending on the signal of
% dH)

dH = H - H_prev;

if dH > 0
    delta = 1; % Ascending branch of the hysteresis loop
elseif dH < 0
    delta = -1; % Descending branch of the hysteresis loop
else
    delta = delta_prev;  % Local inversion condition / stationary dH
end
delta_prev = delta;

% Effective Magnetic Field (He) calculation
He = H + alpha*M_state;
x = He/a;

% Anhysteretic Magnetization (Man) based on classical Langevin formulation
% Includes Taylor series expansion to prevent numerical singularity as He -> 0
if abs(x) < 1e-6
    Man = M_sat * (x/3);
    dMandHe = (M_sat/a) * (1/3);
else
    Man = M_sat * (coth(x) - 1/x);
    % Analytical derivative of the Langevin function with respect to the effective field (dMan/dHe)
    dMandHe = (M_sat/a) * (-1/(sinh(x)^2) + 1/(x^2)); 
end

% Differential components of the magnetic variation rate

dMirr_dH=(Man-M_state)/(delta*k-alpha*(Man-M_state)); % Irreversible contribution
dMrev_dH = c*dMandHe; % Reversible contribution

% Combined differential equation (Standard Jiles-Atherton formulation)
dM_dH=(1/(c+1))*(dMirr_dH + dMrev_dH);

% Magnetization state update via discrete numerical integration (Euler's Method)
M_state = M_state + dM_dH*dH;

% Physical constraint for material coercive saturation
if M_state > M_sat
    M_state = M_sat;
elseif M_state < -M_sat
    M_state = -M_sat;
end

% Output variable assignment
M = M_state;           % [A/m]
B = mu0*(H + M);       % [T]

% History update for the subsequent integration step
H_prev = H;
delta_prev = delta;
end
