%% ASEN 3128 - Lab 6
% Skylar Clark
% 28 April 2022

clear
clc

%% Constants

g = 9.81;           %[m/s^2]

lb_to_N = 4.44822;  %[N/lb]
ft_to_m = 0.3048;   %[m/ft]

S = 5500*ft_to_m^2;         %[m^2]
b = 195.68*ft_to_m;         %[m]
c = 27.31*ft_to_m;          %[m]
h = 0.25;

alt = 20000;        %[ft]
rho = 0.652694;     %[kg/m^3]
M = 0.5;            
u_0 = 518*ft_to_m;    %[m/s]
W = 6.366e5*lb_to_N;%[N]
m = W/g;            %[kg]
I_x = 1.82e7*1.3558179619;       %[kg-m^2]
I_y = 3.31e7*1.3558179619;       %[kg-m^2]
I_z = 4.97e7*1.3558179619;       %[kg-m^2]
I_xz = 9.70e5*1.3558179619;      %[kg-m^2]
zeta = -6.8;        %[deg]
theta_0 = 0;
C_D = 0.040;        
C_W_0 = W / (.5*rho*u_0^2*S);

%% Problem 1

tab6_6 = [-0.8771 -0.2797 0.1946; 0 -0.3295 -0.04073; 0 0.304 -0.2737];

Stab_Derivs_SI = [.5*rho*u_0*S*tab6_6(1,1) .5*rho*u_0*b*S*tab6_6(1,2) .5*rho*u_0*b*S*tab6_6(1,3);
          .25*rho*u_0*b*S*tab6_6(2,1) .25*rho*u_0*b^2*S*tab6_6(2,2) .25*rho*u_0*b^2*S*tab6_6(2,3);
          .25*rho*u_0*b*S*tab6_6(3,1) .25*rho*u_0*b^2*S*tab6_6(3,2) .25*rho*u_0*b^2*S*tab6_6(3,3)];

Stab_Derivs_Rotated = [Stab_Derivs_SI(1,1) Stab_Derivs_SI(1,2)*cosd(zeta)-Stab_Derivs_SI(1,3)*sind(zeta) Stab_Derivs_SI(1,3)*cosd(zeta)+Stab_Derivs_SI(1,2)*sind(zeta);
                       Stab_Derivs_SI(2,1)*cosd(zeta)-Stab_Derivs_SI(3,1)*sind(zeta) Stab_Derivs_SI(2,2)*cosd(zeta)^2-(Stab_Derivs_SI(3,2)+Stab_Derivs_SI(2,3))*sind(zeta)*cosd(zeta)+Stab_Derivs_SI(3,3)*sind(zeta)^2 Stab_Derivs_SI(2,3)*cosd(zeta)^2-(Stab_Derivs_SI(3,3)-Stab_Derivs_SI(2,2))*sind(zeta)*cosd(zeta)-Stab_Derivs_SI(3,2)*sind(zeta)^2;
                       Stab_Derivs_SI(3,1)*cosd(zeta)+Stab_Derivs_SI(2,1)*sind(zeta) Stab_Derivs_SI(3,2)*cosd(zeta)^2-(Stab_Derivs_SI(3,3)-Stab_Derivs_SI(2,2))*sind(zeta)*cosd(zeta)-Stab_Derivs_SI(2,3)*sind(zeta)^2 Stab_Derivs_SI(3,3)*cosd(zeta)^2+(Stab_Derivs_SI(3,2)+Stab_Derivs_SI(2,3))*sind(zeta)*cosd(zeta)+Stab_Derivs_SI(2,2)*sind(zeta)^2];
                   
Y_v = Stab_Derivs_Rotated(1,1);
Y_p = Stab_Derivs_Rotated(2,1);
Y_r = Stab_Derivs_Rotated(3,1);

L_v = Stab_Derivs_Rotated(1,2);
L_p = Stab_Derivs_Rotated(2,2);
L_r = Stab_Derivs_Rotated(3,2);
            
N_v = Stab_Derivs_Rotated(1,3);
N_p = Stab_Derivs_Rotated(2,3);
N_r = Stab_Derivs_Rotated(3,3);

%% Problem 2

I_x_star = I_x*cosd(zeta)^2+I_z*sind(zeta)^2+I_xz*sind(2*zeta);
I_z_star = I_x*sind(zeta)^2+I_z*cosd(zeta)^2-I_xz*sind(2*zeta);
I_xz_star = -.5*(I_x-I_z)*sind(2*zeta)-I_xz*(sind(zeta)^2-cosd(zeta)^2);

Gamma = (I_x_star * I_z_star) - (I_xz_star)^2;
Gamma3 = I_z_star / Gamma;
Gamma4 = I_xz_star / Gamma;
Gamma8 = I_x_star / Gamma;
A11 = Y_v / m;
A12 = Y_p / m;
A13 = (Y_r / m) - u_0;
A14 = g * cosd(theta_0);
A21 = (Gamma3 * L_v) + (Gamma4 * N_v);
A22 = (Gamma3 * L_p) + (Gamma4 * N_p);
A23 = (Gamma3 * L_r) + (Gamma4 * N_r);
A24 = 0;
A31 = (Gamma4 * L_v) + (Gamma8 * N_v);
A32 = (Gamma4 * L_p) + (Gamma8 * N_p);
A33 = (Gamma4 * L_r) + (Gamma8 * N_r);
A34 = 0;
A41 = 0;
A42 = 1;
A43 = tand(theta_0);
A44 = 0;

A = [A11 A12 A13 A14 ; A21 A22 A23 A24 ; A31 A32 A33 A34 ; A41 A42 A43 A44];

%% Problem 3

[eigenvectors,eigenvalues] = eig(A);

DR_eval = eigenvalues(1,1);
Roll_eval = eigenvalues(3,3);
Spiral_eval = eigenvalues(4,4);

DR_tau = -1/real(DR_eval);
Roll_tau = -1/Roll_eval;
Spiral_tau = -1/Spiral_eval;

wn = sqrt(real(DR_eval)^2+imag(DR_eval)^2);
zeta = -real(DR_eval)/wn;


%% Problem 5

tab7_3 = [0 -1.368E-2 -1.973E-4; 0.1146 6.976E-3 -0.1257];

Control_Derivs = [tab7_3(1,1)*.5*rho*u_0^2*S tab7_3(1,2)*.5*rho*u_0^2*S*b tab7_3(1,3)*.5*rho*u_0^2*S*b;
                  tab7_3(2,1)*.5*rho*u_0^2*S tab7_3(2,2)*.5*rho*u_0^2*S*b tab7_3(2,3)*.5*rho*u_0^2*S*b];
         
Y_delta_a = Control_Derivs(1,1);
Y_delta_r = Control_Derivs(2,1);

L_delta_a = Control_Derivs(1,2);
L_delta_r = Control_Derivs(2,2);

N_delta_a = Control_Derivs(1,3);
N_delta_r = Control_Derivs(2,3);

              
Ix_prime = ((I_x_star * I_z_star) - (I_xz_star)^2 ) / I_z_star;
Iz_prime = ((I_x_star * I_z_star) - (I_xz_star)^2 ) / I_x_star;
Izx_prime = (I_xz_star) / ((I_x_star * I_z_star) - (I_xz_star)^2);

B11 = (Y_delta_a) / m;
B12 = (Y_delta_r) / m;
B21 = (L_delta_a/ Ix_prime) + (Izx_prime * N_delta_a);
B22 = (L_delta_r/ Ix_prime) + (Izx_prime * N_delta_r);
B31 = (Izx_prime * L_delta_a) + (N_delta_a / Iz_prime);
B32 = (Izx_prime * L_delta_r) + (N_delta_r / Iz_prime);
B41 = 0;
B42 = 0;
B = [B11 B12 ; B21 B22 ; B31 B32 ; B41 B42];

A15 = 0;
A16 = 0;

A25 = 0;
A26 = 0;

A35 = 0;
A36 = 0;

A45 = 0;
A46 = 0;

A51 = 0;
A52 = 0;
A53 = secd(theta_0);
A54 = 0;
A55 = 0;
A56 = 0;
A61 = 1;
A62 = 0;
A63 = 0;
A64 = 0;
A65 = u_0 * cosd(theta_0);
A66 = 0;
A_aug = [A11 A12 A13 A14 A15 A16 ; 
         A21 A22 A23 A24 A25 A26 ; 
         A31 A32 A33 A34 A35 A36 ;
         A41 A42 A43 A44 A45 A46 ;
         A51 A52 A53 A54 A55 A56 ;
         A61 A62 A63 A64 A65 A66] ;

%B-lat -> 5 (6 by 2) aka B_aug
B51 = 0;
B52 = 0;
B61 = 0;
B62 = 0;
B_aug = [B11 B12 ; B21 B22 ; B31 B32 ; B41 B42 ; B51 B52 ; B61 B62];
