function F_d_atm = drag_force_atm(r_eci, v_eci)

   % garantir vetores coluna
    r_eci = r_eci(:);
    v_eci = v_eci(:);

A=0.01; %m^2 (Num CubeSat de 1 Unidade a área é 10 x 10 cm^2
Cd=2.2; %Em LEO e para um satélite cúbito (1U), o valor do coef de arrasto, Cd, varia de 2 a 2.5
R_Earth = 6.371e6;
h_ref= 3.5e5; %m
%h2= 3.75e5; m  
h =norm(r_eci) - R_Earth; 
H=5e4;
rho350 =1.54e-11; %kg/m^3 Valor Tabelado
rho375 = rho350*exp(-(h-h_ref)/H);

w_z_Earth = 7.2921159e-5;
w_atm_Earth = [0; 0; w_z_Earth];

v_atm_eci = cross(w_atm_Earth, r_eci);
v_rel = v_eci - v_atm_eci;
v_rel_norm =norm(v_rel);

F_d_atm=0.5*rho375*Cd*A*v_rel_norm*v_rel;

end
