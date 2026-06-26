function a = gravitical_acelaration(r)

% Computes the gravitational acceleration acting on a satellite
% based on its position vector in the ECI reference frame.

M_Earth=5.972e24; % Earth mass [kg]

G=6.674e-11; % Gravitational constant [m^3/(kgs^2)]

% Magnitude of the position vector (3x1 - ECI frame)

r_norm = sqrt(r(1)^2 + r(2)^2 + r(3)^2);

% Gravitational acceleration vector

a = -G*M_Earth * r / (r_norm^3);

end