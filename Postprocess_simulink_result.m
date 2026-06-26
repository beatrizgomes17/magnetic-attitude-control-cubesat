%Constants
%Nota: o satélite demora mais ou menos 5515 s a fzr uma orbita
%Pos maxima em x -> 1382 para T sample = 1s

%Time sample (it respects Nyquist (B dot))

Ts=0.1; 

% Inertial Matrix of the Kitsat (A good aproximation based on a thesis of a
% 1U CubeSat Satellite.

%J=[1.67e-3 0 0;
 %   0 1.67e-3 0;
    %0 0 1.67e-3];
   
 %J= [1.062e-3 4.094e-8 3.487e-6;
    %4.094e-8 1.074e-3 2.763e-7;
    %3.487e-6 2.763e-7 5.236e-4];

 J= [1.062e-3 4.094e-8 3.487e-6;
    4.094e-8 1.074e-3 2.763e-7;
    3.487e-6 2.763e-7 1.03e-3];
% Earth's Mass in Kg

M_Earth = 5.972e24;  

% Earth's Radius in m

R_Earth = 6.371e6;   

%Gravitacional Constant

G = 6.674e-11; 

% Satellite' Altitude: 375 km.

altitude = 3.75e5;   

% Initial Position Vector

r0_y= R_Earth+altitude;
r0_vect = [0; r0_y ; 0];

% Magnitude of the Initial Position Vector 

r0_mag = norm(r0_vect); 

% Initial Velocity vector

v0_x = sqrt(G*M_Earth/r0_mag);
v0_vect = [v0_x; 0; 0];

% Definition of B dot gain

k_bdot=5e4; %2.5e5 %0.7e5

%Definir o numero de voltas da bobina e a área de cada espira.
%diametro do fio de cobre: 0.25 mm (mas este fio so suporta ate 0,147 A!)
%N=264; %considerando um comprimento do solenoide de 66 mm, mas
%considerando um erro de 10%
%N=238;
N=210; %considerando um diâmetro de fio de 0,81 mm e um erro de 10%.
L=66.0e-3;
raio = 5.75e-3; %raio do solenoide
A=pi*raio^2; %considerando um solenoide se secção circular
Vcore=A*L;
mu0=4*pi*1e-7;%permeabilidade no vacuo
%Definir corrente elétrica que delimita o momento magnético gerado pelas
%bobinas. Considerei 1 A visto (1.5A) ser o valor máximo fornecido pelo KitSat
I_max=0.6; %Máxima corrente eletrica do magnetorquer
I_min=-0.6;
m_max=N * I_max * A; %isto será o limite físico da minha bobina
m_min=N * I_min * A;

%Determinar o valor de I a aplicar ao magnetorquer em cada instante de modo
%I= out.Corrente.signals.values;
%Ix=I(:,1);
%Iy=I(:,2);
%Iz=I(:,3);

%Validacao do controlador b dot

%Validation= out.verificacao_b_Dot.signals.values;
%No Simulink estou a utilizar  Structure With Time

%How to save values of Simulink in Matlab

r=out.r_log.signals.values;
t=out.r_log.time; % time vector with a time step of 0,1 seconds (Time sample defined)
rx=r(:,1); %valores de x contidos no meu vetor posicao
ry=r(:,2); %valores de y contidos no meu vetor posicao
rz=r(:,3);

% Satellite Position along the Orbit - 2D plot

figure;

plot(rx,ry,  'LineWidth',2);
xlabel('Position in x (m)');
ylabel('Position in y (m)');
grid on;
axis equal;
title('Satellite Position - 2D Orbit');

% Satellite Position along the Orbit - 3D plot

figure;

plot3(rx,ry,rz,'k', 'LineWidth',1.8);

xlabel('Position in x (m)');
ylabel('Position in y (m)');
zlabel('Position in z (m)');
grid on;
axis equal;
title('Satellite Position - 3D Orbit');
hold on;

% Earth's scheme (sphere)

[xe, ye, ze] = sphere(60);
Ra = 6371e3;   % raio da Terra
surf(Ra*xe, Ra*ye, Ra*ze, 'FaceColor',[0.3 0.6 1], 'EdgeColor','none', 'FaceAlpha',0.7);
camlight; lighting gouraud;  % iluminação da esfera

% Magnetic field along time - 3D plot

B= out.B.signals.values;

Bx=B(:,1); %values of x of the magnetic field vector
By=B(:,2); %values of y of the magnetic field vector
Bz=B(:,3); %values of z of the magnetic field vector

figure;

plot3(Bx,By,Bz, 'k', 'LineWidth', 2);

xlabel('B_x (T)');
ylabel('B_y (T)');
zlabel('B_z (T)');
title('Variation of the magnetic field along the orbit');
grid on;
axis equal;
box on;

% Magnetic Field magnitude along time

Bnorm = sqrt(Bx.^2 + By.^2 + Bz.^2);
figure;
plot(t, Bnorm, 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('|B| (T)');
title('Modulus of the magnetic field as a function of time');

% Torque magnitude along time

T= out.torque.signals.values;

Tx=T(:,1); %values of x of the torque field vector
Ty=T(:,2); %values of y of the torque field vector
Tz=T(:,3); %values of z of the torque field vector

%B_JA = out.magneticB.signals.values;
torqueMagnitude = sqrt(Tx.^2 + Ty.^2 + Tz.^2);
figure;
plot(t, torqueMagnitude, 'LineWidth', 2);
plot(t, Tx, t,Ty,t,Tz, 'LineWidth', 2);

grid on;
xlabel('Time (s)');
ylabel('Torque Magnitude (Nm)');
title('Torque Magnitude as a function of time');

% Magnetic Moment plot along time 

m= out.magneticmoment_real.signals.values;

mx=m(:,1);
my=m(:,2);
mz=m(:,3);

figure;
plot(t, mx, t,my,t,mz, 'LineWidth', 2);
grid on;
xlabel('Time (s)');
ylabel('Magnetic Moment (Am^2)');
title('Magnetic Moment as a function of time');

% Angular Velocity along time

% This part of the code was done in order to eliminate the first value of
% the angular velocity, because it is not real, as it results of the
% calculation involving B, and at the beginning there is no previous value
% of B. A buffer time vector was also created, eliminating the first
% element, in order to obtain same size vectores.

w=out.w.signals.values;
w_body=out.wbody.signals.values;
%----------
w(1) = [];
tbuffer=t;
tbuffer(1)=[];
%----------
modulo_w = sqrt(sum(w_body.^2, 2));
figure;
plot(t,modulo_w,'LineWidth', 2 );
xlabel('Time (s)');
ylabel('Modulus of Angular Velocity (rads-1)');
title('Modulus of Angular Velocity as a function of Time');

figure;
plot(t,w_body, 'LineWidth', 2);
grid on;
xlabel('Time (s)');
ylabel('Angular Velocity (rads-1)');
title('Angular Velocity as a function of Time');

%Stability criterion of Lyapunov

% Candidate Function: Rotational Kinetic Energy

% V=0.5*wT*J*w
% dV= wT*torque

dV=out.dV.signals.values;

figure;
plot(t,dV, 'LineWidth', 2);
grid on;
xlabel('Time (s)');
ylabel('derivative of rotational Kinetic Energy dV ');
title('Stabilization');

I=out.current.signals.values;
Ix=I(:,1); %valores de x contidos no meu vetor posicao
Iy=I(:,2); %valores de y contidos no meu vetor posicao
Iz=I(:,3);

figure;
plot(t, Ix, t,Iy,t,Iz, 'LineWidth', 2);
grid on;
xlabel('Time (s)');
ylabel('Current (A)');
title('Current as a function of time');

