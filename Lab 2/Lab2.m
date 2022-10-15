% Group 24: Izaak Beusmans, Kate Boykin, Caleb Bristol, Skylar Clark
% ASEN 3128
% Lab 2 - Quadrotor Dynamics
% 16 Feb 2022

clear
clc

%% Problem 1

% Constants
g = 9.81;       %[m/s^2]
m = 0.068;      %[kg]
r = 0.060;      %[m]
k_m = 0.0024;   %[Nm/N]
I_x = 6.8E-5;   %[kg*m^2]
I_y = 9.2E-5;   %[kg*m^2]
I_z = 1.35E-4;  %[kg*m^2]
nu = 1E-3;      %[N/(m/s)^2]
mu = 2E-6;      %[Nm/(rad/s)^2]

tspan = [0 100];%[s]

% Solved by using the w_dot quadrotor equation and omitting aerodynamic forces
Zc_1 = -m*g;

% Solved by using systems of equations on control forces/moments with Lc,
% Mc, and Nc = 0
f1 = m*g/4;     %[N]
f2 = m*g/4;
f3 = m*g/4;
f4 = m*g/4;

% Setting up control forces/moments vectors
Fc_1 = [0; 0; Zc_1];
Gc_1 = [(r/sqrt(2))*(-f1-f2+f3+f4); (r/sqrt(2))*(f1-f2-f3+f4); k_m*(f1-f2+f3-f4)];

% Initial Conditions vector (all zeros for steady hover)
IC_1 = [0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0];
         
% Solve using ODE45
[t_out1,var_out1] = ode45(@(t,var) AircraftEOM_No_Aero(t,var,g,m,nu,mu,Fc_1,Gc_1),tspan,IC_1);

% Plotting equilibrium results
label_names = ["x_E [m]" "y_E [m]" "z_E [m]" "\phi [rad]" "\theta [rad]" "\psi [rad]" "u^E [m/s]" "v^E [m/s]" "w^E [m/s]" "p [rad/s]" "q [rad/s]" "r [rad/s]"];
figure(1)
for i = 1:12
    subplot(4,3,i)
    plot(t_out1,var_out1(:,i),'LineWidth',3)
    ylabel(sprintf("%s",label_names(i)),'FontSize',12)
    xlabel("Time [s]",'FontSize',12)
end
sgtitle("Problem 1: Trim State Simulation (No Aerodynamic Forces)",'FontSize',16)

%% Problem 2a

% Verifying that steady hover initial conditions works when adding
% aerodynamic forces with ODE45
[t_out2a,var_out2a] = ode45(@(t,var) AircraftEOM(t,var,g,m,nu,mu,Fc_1,Gc_1),tspan,IC_1);

% Plotting equilibrium results
figure(2)
for i = 1:12
    subplot(4,3,i)
    plot(t_out2a,var_out2a(:,i),'LineWidth',3)
    ylabel(sprintf("%s",label_names(i)),'FontSize',12)
    xlabel("Time [s]",'FontSize',12)
end
sgtitle("Problem 2a: Trim State Simulation (With Aerodynamic Forces)",'FontSize',16)

%% Problem 2b

% Constant inertial velocity
v_east = 5;     %[m/s]

% Solved using v_dot quadrotor equations when v_dot = 0
phi_b = asin((nu*v_east^2)/(m*g));   %[rad]

% Using only phi in the DCM since theta and psi = 0
inertial_to_body_DCM = [1 0 0; 0 cos(phi_b) sin(phi_b); 0 -sin(phi_b) cos(phi_b)];

% Converting inertial velocity to body frame velocity
velocity_body_2b = inertial_to_body_DCM*[0 v_east 0]';  %[m/s]

% Calculating for aerodynamic force Z
Z_2b = -nu*v_east*velocity_body_2b(3);   %[N]

% Solved by using w_dot quadrotor equation
Zc_2b = -m*g*cos(phi_b) - Z_2b;             %[N]

% Solved by using systems of equations on control forces/moments with Lc,
% Mc, and Nc = 0
f1 = (m*g*cos(phi_b)+Z_2b)/4;            %[N]
f2 = (m*g*cos(phi_b)+Z_2b)/4;
f3 = (m*g*cos(phi_b)+Z_2b)/4;
f4 = (m*g*cos(phi_b)+Z_2b)/4;

% Setting up control forces/moments
Fc_2b = [0; 0; Zc_2b];
Gc_2b = [(r/sqrt(2))*(-f1-f2+f3+f4); (r/sqrt(2))*(f1-f2-f3+f4); k_m*(f1-f2+f3-f4)];

% Initial Conditions Vector for v^E_E = 5, psi = 0
IC_2b = [0; 0; 0; phi_b; 0; 0; 0;velocity_body_2b(2); velocity_body_2b(3); 0; 0; 0];

% Solve using ODE45
[t_out2b,var_out2b] = ode45(@(t,var) AircraftEOM(t,var,g,m,nu,mu,Fc_2b,Gc_2b),tspan,IC_2b);

% Plot results
figure(3)
for i = 1:12
    subplot(4,3,i)
    plot(t_out2b,var_out2b(:,i),'Linewidth',3)
    grid on;
    ylabel(sprintf("%s",label_names(i)),'FontSize',12)
    xlabel("Time [s]",'FontSize',12)
end
sgtitle("Problem 2b: Trim State Simulation (v^E_E = 5, \psi = 0)",'FontSize',16)

%% Problem 2c

% constant psi value
psi_c = pi/2;       %[rad]

% Solve for aerodynamic force X (u velocity will remain effectively as inertial velocity)
X = -nu*v_east^2;   %[N]

% Solved using u_dot quadrotor equation
theta_c = asin(X/(m*g));    %[rad]

% Find DCM to convert from inertial velocity to body frame velocity
DCM1 = [cos(theta_c) 0 -sin(theta_c); 0 1 0; sin(theta_c) 0 cos(theta_c)];
DCM2 = [cos(psi_c) sin(psi_c) 0; -sin(psi_c) cos(psi_c) 0; 0 0 1];

DCM_total = DCM1*DCM2;

% Convert to body frame
velocity_body_2c = DCM_total*[0 v_east 0]';

% Solve for aerodynamic force Z
Z_2c = -nu*v_east*velocity_body_2c(3);

% Solved using w_dot equation
Zc_2c = -m*g*cos(theta_c) - Z_2c;

% Solved by using systems of equations on control forces/moments with Lc,
% Mc, and Nc = 0
f1 = (m*g*cos(theta_c)+Z_2c)/4;     %[N]
f2 = (m*g*cos(theta_c)+Z_2c)/4;
f3 = (m*g*cos(theta_c)+Z_2c)/4;
f4 = (m*g*cos(theta_c)+Z_2c)/4;

% Setting up control forces/moments
Fc_2c = [0; 0; Zc_2b];
Gc_2c = [(r/sqrt(2))*(-f1-f2+f3+f4); (r/sqrt(2))*(f1-f2-f3+f4); k_m*(f1-f2+f3-f4)];

% Initial Conditions vector for v^E_E = 5, psi = pi/2
IC_2c = [0; 0; 0; 0; theta_c; psi_c; velocity_body_2c(1); velocity_body_2c(2); velocity_body_2c(3); 0; 0; 0];

% Solve using ODE45
[t_out2c,var_out2c] = ode45(@(t,var) AircraftEOM(t,var,g,m,nu,mu,Fc_2c,Gc_2c),tspan,IC_2c);

% Plot results
figure(4)
for i = 1:12
    subplot(4,3,i)
    plot(t_out2c,var_out2c(:,i),'Linewidth',3)
    grid on;
    ylabel(sprintf("%s",label_names(i)),'FontSize',12)
    xlabel("Time [s]",'FontSize',12)
    if (i == 1) || (i == 8)
        ylim([-1 1])
    end
end
sgtitle("Problem 2c: Trim State Simulation (v^E_E = 5, \psi = \pi/2)",'FontSize',16)

%% ODE Functions

function dvar_dt = AircraftEOM_No_Aero(t,var,g,m,nu,mu,Fc,Gc)
% Group 24
% Members: Skylar Clark, Caleb Bristol, Kate Boykin, Izaak Beusmans
% INPUTS: t is scalar time
%         var is a column vector of the aircraft state
%         g is scalar gravity
%         m is scalar mass
%         nu is the scalar aerodynamic force coefficient
%         mu is the scalar aerodynamic moment coefficient
%         Fc is a column vector of Body-Frame Control Forces
%         Gc is a column vector of Body-Frame Control Moments

pos = var(1:3);
ang = var(4:6);
vel = var(7:9);
ang_vel = var(10:12);

I_B = [6.8E-5; 9.2E-5; 1.35E-4];

% Calculate position dynamics
pos_DCM =   [cos(ang(2))*cos(ang(3)) sin(ang(1))*sin(ang(2))*cos(ang(3))-cos(ang(1))*sin(ang(3)) cos(ang(1))*sin(ang(2))*cos(ang(3))+sin(ang(1))*sin(ang(3));
             cos(ang(2))*sin(ang(3)) sin(ang(1))*sin(ang(2))*sin(ang(3))+cos(ang(1))*cos(ang(3)) cos(ang(1))*sin(ang(2))*sin(ang(3))-sin(ang(1))*cos(ang(3));
             -sin(ang(2)) sin(ang(1))*cos(ang(2)) cos(ang(1))*cos(ang(2))];

dpos_dt = pos_DCM*vel;

% Calculate angular dynamics
ang_DCM =   [1 sin(ang(1))*tan(ang(2)) cos(ang(1))*tan(ang(2));
             0 cos(ang(1)) -sin(ang(1));
             0 sin(ang(1))*sec(ang(2)) cos(ang(1))*sec(ang(2))];
         
dang_dt = ang_DCM*ang_vel;

% Calculate velocity dynamics

vel_term_1 =    [ang_vel(3)*vel(2)-ang_vel(2)*vel(3);
             ang_vel(1)*vel(3)-ang_vel(3)*vel(1);
             ang_vel(2)*vel(1)-ang_vel(1)*vel(2)];
       
vel_term_2 =    g * [-sin(ang(2));
                 cos(ang(2))*sin(ang(1));
                 cos(ang(2))*cos(ang(1))];
             
vel_term_3 =    Fc/m;

dvel_dt = vel_term_1 + vel_term_2 + vel_term_3;

% Calculate angular velocity dynamics

ang_vel_term_1 =    [(I_B(2)-I_B(3))*ang_vel(2)*ang_vel(3)/I_B(1);
                     (I_B(3)-I_B(1))*ang_vel(1)*ang_vel(3)/I_B(2);
                     (I_B(1)-I_B(2))*ang_vel(1)*ang_vel(2)/I_B(3)];
                 
ang_vel_term_2 =    Gc./I_B;

dangvel_dt = ang_vel_term_1 + ang_vel_term_2;

dvar_dt = [dpos_dt;dang_dt;dvel_dt;dangvel_dt];

end

function dvar_dt = AircraftEOM(t,var,g,m,nu,mu,Fc,Gc)
% Group 24
% Members: Skylar Clark, Caleb Bristol, Kate Boykin, Izaak Beusmans
% INPUTS: t is scalar time
%         var is a column vector of the aircraft state
%         g is scalar gravity
%         m is scalar mass
%         nu is the scalar aerodynamic force coefficient
%         mu is the scalar aerodynamic moment coefficient
%         Fc is a column vector of Body-Frame Control Forces
%         Gc is a column vector of Body-Frame Control Moments

pos = var(1:3);
ang = var(4:6);
vel = var(7:9);
ang_vel = var(10:12);

airspeed = norm(vel);
omega_mag = norm(ang_vel);

I_B = [6.8E-5; 9.2E-5; 1.35E-4];

% Calculate position dynamics
pos_DCM =   [cos(ang(2))*cos(ang(3)) sin(ang(1))*sin(ang(2))*cos(ang(3))-cos(ang(1))*sin(ang(3)) cos(ang(1))*sin(ang(2))*cos(ang(3))+sin(ang(1))*sin(ang(3));
             cos(ang(2))*sin(ang(3)) sin(ang(1))*sin(ang(2))*sin(ang(3))+cos(ang(1))*cos(ang(3)) cos(ang(1))*sin(ang(2))*sin(ang(3))-sin(ang(1))*cos(ang(3));
             -sin(ang(2)) sin(ang(1))*cos(ang(2)) cos(ang(1))*cos(ang(2))];

dpos_dt = pos_DCM*vel;

% Calculate angular dynamics
ang_DCM =   [1 sin(ang(1))*tan(ang(2)) cos(ang(1))*tan(ang(2));
             0 cos(ang(1)) -sin(ang(1));
             0 sin(ang(1))*sec(ang(2)) cos(ang(1))*sec(ang(2))];
         
dang_dt = ang_DCM*ang_vel;

% Calculate velocity dynamics

vel_term_1 =    [ang_vel(3)*vel(2)-ang_vel(2)*vel(3);
             ang_vel(1)*vel(3)-ang_vel(3)*vel(1);
             ang_vel(2)*vel(1)-ang_vel(1)*vel(2)];
       
vel_term_2 =    g * [-sin(ang(2));
                 cos(ang(2))*sin(ang(1));
                 cos(ang(2))*cos(ang(1))];
             
vel_term_3 =    -nu*airspeed/m * vel;
             
vel_term_4 =    Fc/m;

dvel_dt = vel_term_1 + vel_term_2 + vel_term_3 + vel_term_4;

% Calculate angular velocity dynamics

ang_vel_term_1 =    [(I_B(2)-I_B(3))*ang_vel(2)*ang_vel(3)/I_B(1);
                     (I_B(3)-I_B(1))*ang_vel(1)*ang_vel(3)/I_B(2);
                     (I_B(1)-I_B(2))*ang_vel(1)*ang_vel(2)/I_B(3)];
                 
ang_vel_term_2 =    -mu*omega_mag*ang_vel./I_B;
                 
ang_vel_term_3 =    Gc./I_B;

dangvel_dt = ang_vel_term_1 + ang_vel_term_2 + ang_vel_term_3;

dvar_dt = [dpos_dt;dang_dt;dvel_dt;dangvel_dt];
end