%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ASEN 3128: Lab 3
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%
clear
clc

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

%% Problem 3

    tspan = [0 10];%[s]
    
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
    IC_a = [0; 0; 0; deg2rad(5); 0; 0; 0; 0; 0; 0; 0; 0];
    IC_b = [0; 0; 0; 0; deg2rad(5); 0; 0; 0; 0; 0; 0; 0];
    IC_c = [0; 0; 0; 0; 0; deg2rad(5); 0; 0; 0; 0; 0; 0];
    IC_d = [0; 0; 0; 0; 0; 0; 0; 0; 0; 0.1; 0; 0];
    IC_e = [0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0.1; 0];
    IC_f = [0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0.1];

    % Solve using ODE45
    [t_out_a,var_out_a] = ode45(@(t,var) nonlinearEOM(t,var,g,m,nu,mu,Fc_1,Gc_1),tspan,IC_a);
    [t_out_b,var_out_b] = ode45(@(t,var) nonlinearEOM(t,var,g,m,nu,mu,Fc_1,Gc_1),tspan,IC_b);
    [t_out_c,var_out_c] = ode45(@(t,var) nonlinearEOM(t,var,g,m,nu,mu,Fc_1,Gc_1),tspan,IC_c);
    [t_out_d,var_out_d] = ode45(@(t,var) nonlinearEOM(t,var,g,m,nu,mu,Fc_1,Gc_1),tspan,IC_d);
    [t_out_e,var_out_e] = ode45(@(t,var) nonlinearEOM(t,var,g,m,nu,mu,Fc_1,Gc_1),tspan,IC_e);
    [t_out_f,var_out_f] = ode45(@(t,var) nonlinearEOM(t,var,g,m,nu,mu,Fc_1,Gc_1),tspan,IC_f);
    
    % Plotting
    f_c_a = [f1*ones(length(t_out_a),1) f2*ones(length(t_out_a),1) f3*ones(length(t_out_a),1) f4*ones(length(t_out_a),1)];
    f_c_b = [f1*ones(length(t_out_b),1) f2*ones(length(t_out_b),1) f3*ones(length(t_out_b),1) f4*ones(length(t_out_b),1)];
    f_c_c = [f1*ones(length(t_out_c),1) f2*ones(length(t_out_c),1) f3*ones(length(t_out_c),1) f4*ones(length(t_out_c),1)];
    f_c_d = [f1*ones(length(t_out_d),1) f2*ones(length(t_out_d),1) f3*ones(length(t_out_d),1) f4*ones(length(t_out_d),1)];
    f_c_e = [f1*ones(length(t_out_e),1) f2*ones(length(t_out_e),1) f3*ones(length(t_out_e),1) f4*ones(length(t_out_e),1)];
    f_c_f = [f1*ones(length(t_out_f),1) f2*ones(length(t_out_f),1) f3*ones(length(t_out_f),1) f4*ones(length(t_out_f),1)];
    
    PlotAircraftSim(t_out_a, var_out_a',f_c_a', 1:6, ["-r","non-linear";"-","linear"])
    PlotAircraftSim(t_out_b, var_out_b',f_c_b', 7:12, ["-r","non-linear";"-","linear"])
    PlotAircraftSim(t_out_c, var_out_c',f_c_c', 13:18, ["-r","non-linear";"-","linear"])
    PlotAircraftSim(t_out_d, var_out_d',f_c_d', 19:24, ["-r","non-linear";"-","linear"])
    PlotAircraftSim(t_out_e, var_out_e',f_c_e', 25:30, ["-r","non-linear";"-","linear"])
    PlotAircraftSim(t_out_f, var_out_f',f_c_f', 31:36, ["-r","non-linear";"-","linear"])
    
 
%% Problem 4 

    tspan = [0 10];%[s]
    
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
    IC_a = [0; 0; 0; deg2rad(5); 0; 0; 0; 0; 0; 0; 0; 0];
    IC_b = [0; 0; 0; 0; deg2rad(5); 0; 0; 0; 0; 0; 0; 0];
    IC_c = [0; 0; 0; 0; 0; deg2rad(5); 0; 0; 0; 0; 0; 0];
    IC_d = [0; 0; 0; 0; 0; 0; 0; 0; 0; 0.1; 0; 0];
    IC_e = [0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0.1; 0];
    IC_f = [0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0.1];

    % Solve using ODE45
    [t_out_a4,var_out_a4] = ode45(@(t,var) linearEOM(t,var,g,m,nu,mu,Fc_1,Gc_1),tspan,IC_a);
    [t_out_b4,var_out_b4] = ode45(@(t,var) linearEOM(t,var,g,m,nu,mu,Fc_1,Gc_1),tspan,IC_b);
    [t_out_c4,var_out_c4] = ode45(@(t,var) linearEOM(t,var,g,m,nu,mu,Fc_1,Gc_1),tspan,IC_c);
    [t_out_d4,var_out_d4] = ode45(@(t,var) linearEOM(t,var,g,m,nu,mu,Fc_1,Gc_1),tspan,IC_d);
    [t_out_e4,var_out_e4] = ode45(@(t,var) linearEOM(t,var,g,m,nu,mu,Fc_1,Gc_1),tspan,IC_e);
    [t_out_f4,var_out_f4] = ode45(@(t,var) linearEOM(t,var,g,m,nu,mu,Fc_1,Gc_1),tspan,IC_f);
    
    % Plotting
    f_c_a4 = [f1*ones(length(t_out_a4),1) f2*ones(length(t_out_a4),1) f3*ones(length(t_out_a4),1) f4*ones(length(t_out_a4),1)];
    f_c_b4 = [f1*ones(length(t_out_b4),1) f2*ones(length(t_out_b4),1) f3*ones(length(t_out_b4),1) f4*ones(length(t_out_b4),1)];
    f_c_c4 = [f1*ones(length(t_out_c4),1) f2*ones(length(t_out_c4),1) f3*ones(length(t_out_c4),1) f4*ones(length(t_out_c4),1)];
    f_c_d4 = [f1*ones(length(t_out_d4),1) f2*ones(length(t_out_d4),1) f3*ones(length(t_out_d4),1) f4*ones(length(t_out_d4),1)];
    f_c_e4 = [f1*ones(length(t_out_e4),1) f2*ones(length(t_out_e4),1) f3*ones(length(t_out_e4),1) f4*ones(length(t_out_e4),1)];
    f_c_f4 = [f1*ones(length(t_out_f4),1) f2*ones(length(t_out_f4),1) f3*ones(length(t_out_f4),1) f4*ones(length(t_out_f4),1)];
    
    PlotAircraftSim(t_out_a4, var_out_a4',f_c_a4', 1:6, ["-b","non-linear";"-","linear"])
    PlotAircraftSim(t_out_b4, var_out_b4',f_c_b4', 7:12, ["-b","non-linear";"-","linear"])
    PlotAircraftSim(t_out_c4, var_out_c4',f_c_c4', 13:18, ["-b","non-linear";"-","linear"])
    PlotAircraftSim(t_out_d4, var_out_d4',f_c_d4', 19:24, ["-b","non-linear";"-","linear"])
    PlotAircraftSim(t_out_e4, var_out_e4',f_c_e4', 25:30, ["-b","non-linear";"-","linear"])
    PlotAircraftSim(t_out_f4, var_out_f4',f_c_f4', 31:36, ["-b","non-linear";"-","linear"])
    
%% Problem 5

    tspan = [0 10];%[s]
    
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
    IC_d = [0; 0; 0; 0; 0; 0; 0; 0; 0; 0.1; 0; 0];
    IC_e = [0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0.1; 0];
    IC_f = [0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0.1];

    % Solve using ODE45
    [t_out_d5,var_out_d5] = ode45(@(t,var) nonlinearEOM_fdb(t,var,g,m,nu,mu,Fc_1,Gc_1),tspan,IC_d);
    [t_out_e5,var_out_e5] = ode45(@(t,var) nonlinearEOM_fdb(t,var,g,m,nu,mu,Fc_1,Gc_1),tspan,IC_e);
    [t_out_f5,var_out_f5] = ode45(@(t,var) nonlinearEOM_fdb(t,var,g,m,nu,mu,Fc_1,Gc_1),tspan,IC_f);
    
    % Calculate Respective Control Moments
    k_g = 0.004;
    ang_vel_d5 = var_out_d5(:,10:12);
    ang_vel_e5 = var_out_e5(:,10:12);
    ang_vel_f5 = var_out_f5(:,10:12);
    
    Gc_d5 = Gc_1 .* ones(3,length(t_out_d5)) + [k_g.*-ang_vel_d5(:,1)' ;k_g.*-ang_vel_d5(:,2)';k_g.*-ang_vel_d5(:,3)'];
    Gc_e5 = Gc_1 .* ones(3,length(t_out_e5)) + [k_g.*-ang_vel_e5(:,1)' ;k_g.*-ang_vel_e5(:,2)';k_g.*-ang_vel_e5(:,3)'];
    Gc_f5 = Gc_1 .* ones(3,length(t_out_f5)) + [k_g.*-ang_vel_f5(:,1)' ;k_g.*-ang_vel_f5(:,2)';k_g.*-ang_vel_f5(:,3)'];
    
    % Plotting
    f_c_d5 = ComputeMotorForces(Zc_1*ones(1,length(Gc_d5)), Gc_d5(1,:), Gc_d5(2,:), Gc_d5(3,:), r, k_m);
    f_c_e5 = ComputeMotorForces(Zc_1*ones(1,length(Gc_e5)), Gc_e5(1,:), Gc_e5(2,:), Gc_e5(3,:), r, k_m);
    f_c_f5 = ComputeMotorForces(Zc_1*ones(1,length(Gc_f5)), Gc_f5(1,:), Gc_f5(2,:), Gc_f5(3,:), r, k_m);
    
    PlotAircraftSim(t_out_d5, var_out_d5',f_c_d5, 37:42, ["-k","Feedback Control"])
    PlotAircraftSim(t_out_e5, var_out_e5',f_c_e5, 43:48, ["-k","Feedback Control"])
    PlotAircraftSim(t_out_f5, var_out_f5',f_c_f5, 49:54, ["-k","Feedback Control"])

    
%% Functions

function [] = PlotAircraftSim(time, aircraft_state_array, control_input_array, fig, col)
%% PlotAircraftSim
%
% Function to plot all aircraft variables over time given the state vector,
% control inputs vector, and a column of plotting options
%
% Group 37
% Members:
% -Caleb Bristol
% -Adam Pillari
% -Devon Paris
% -Kushal Kedia

%% Plot Aircraft State Vector

    %% Figure 1: Inertial Position
    figure(fig(1))
    x1 = subplot(3,1,1);
    plot(time,aircraft_state_array(1,:),col(1,1)); hold on
    ylabel('x [m]')
    xlabel('Time [s]')
    legend(col(:,2))
    grid on
    y1 = subplot(3,1,2);
    plot(time,aircraft_state_array(2,:),col(1)); hold on
    ylabel('y [m]')
    xlabel('Time [s]')
    legend(col(:,2))
    grid on
    z1 = subplot(3,1,3);
    plot(time,aircraft_state_array(3,:),col(1)); hold on
    ylabel('z [m]')
    xlabel('Time [s]')
    legend(col(:,2))
    sgtitle('Aircraft Inertial Position')
    grid on
    
    %% Figure 2: Euler Angles
    figure(fig(2))
    phi1 = subplot(3,1,1);
    plot(time,aircraft_state_array(4,:),col(1)); hold on
    ylabel('Roll [rad]')
    xlabel('Time [s]')
    legend(col(:,2))
    grid on
    theta1 = subplot(3,1,2);
    plot(time,aircraft_state_array(5,:),col(1)); hold on
    ylabel('Pitch [rad]')
    xlabel('Time [s]')
    legend(col(:,2))
    grid on
    psi1 = subplot(3,1,3);
    plot(time,aircraft_state_array(6,:),col(1)); hold on
    ylabel('Yaw [rad]')
    xlabel('Time [s]')
    legend(col(:,2))
    sgtitle('Aircraft Orientation [Euler Angles]')
    grid on
    
    %% Figure 3: Inertial Velocity
    figure(fig(3))
    u1 = subplot(3,1,1);
    plot(time,aircraft_state_array(7,:),col(1)); hold on
    ylabel('u [m/s]')
    xlabel('Time [s]')
    legend(col(:,2))
    grid on
    v1 = subplot(3,1,2);
    plot(time,aircraft_state_array(8,:),col(1)); hold on
    ylabel('v [m/s]')
    xlabel('Time [s]')
    legend(col(:,2))
    grid on
    w1 = subplot(3,1,3);
    plot(time,aircraft_state_array(9,:),col(1)); hold on
    ylabel('w [m/s]')
    xlabel('Time [s]')
    legend(col(:,2))
    sgtitle('Aircraft Inertial Velocity')
    grid on
    
    %% Figure 4: Angular Velocity
    figure(fig(4))
    p1 = subplot(3,1,1);
    plot(time,aircraft_state_array(10,:),col(1)); hold on
    ylabel('p [rad/s]')
    xlabel('Time [s]')
    legend(col(:,2))
    grid on
    q1 = subplot(3,1,2);
    plot(time,aircraft_state_array(11,:),col(1)); hold on
    ylabel('q [rad/s]')
    xlabel('Time [s]')
    legend(col(:,2))
    grid on
    r1 = subplot(3,1,3);
    plot(time,aircraft_state_array(12,:),col(1)); hold on
    ylabel('r [rad/s]')
    xlabel('Time [s]')
    legend(col(:,2))
    sgtitle('Aircraft Angular Velocity')
    grid on
    
%% Plot Controls Vector

    %% Figure 5: Control Input Array
    figure(fig(5))
    subplot(2,2,1);
    plot(time,control_input_array(1,:),col(1)); hold on
    title('f1')
    ylabel('Force [N]')
    xlabel('Time [s]')
    legend(col(:,2))
    grid on
    subplot(2,2,2);
    plot(time,control_input_array(2,:),col(1)); hold on
    title('f2')
    ylabel('Force [N]')
    xlabel('Time [s]')
    legend(col(:,2))
    grid on
    subplot(2,2,3);
    plot(time,control_input_array(3,:),col(1)); hold on
    title('f3')
    ylabel('Force [N]')
    xlabel('Time [s]')
    legend(col(:,2))
    grid on
    subplot(2,2,4);
    plot(time,control_input_array(4,:),col(1)); hold on
    title('f4')
    ylabel('Force [N]')
    xlabel('Time [s]')
    legend(col(:,2))
    sgtitle('Aircraft Control Forces [Individual Motors]')
    grid on; grid minor;
    
%% Plot Aircraft Trajectory

    %% Figure 6: 3D Aircraft Trajectory
    figure(fig(6))
    plot3(aircraft_state_array(1,:),aircraft_state_array(2,:),-aircraft_state_array(3,:),col(1)); hold on
    title('Aircraft Trajectory')
    xlabel('x [m]')
    ylabel('y [m]')
    zlabel('z [m]')
    legend(col(:,2))
    grid on; grid minor;
end


    
function dvar_dt = nonlinearEOM_fdb(t,var,g,m,nu,mu,Fc,Gc)
%% nonlinearEOM_fdb
%
% Identical to nonlinearEOM, except includes specified control gain for
% problem 5
%
% Utilized for Problem 5
%
% Group 37
% Members:
% -Caleb Bristol
% -Adam Pillari
% -Devon Paris
% -Kushal Kedia

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

% Angular Control
k_g = 0.004; % Angular Control Gain

% Update Angular Controls
Gc = [Gc(1) + k_g*-ang_vel(1) ;Gc(2) + k_g*-ang_vel(2);Gc(3) + k_g*-ang_vel(3)];


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


function dvar_dt = nonlinearEOM(t,var,g,m,nu,mu,Fc,Gc)
%% nonlinearEOM
%
% Includes the equations for the non-linearized equations of motion,
% utilized in problem 3
%
% Group 37
% Members:
% -Caleb Bristol
% -Adam Pillari
% -Devon Paris
% -Kushal Kedia
%
% Credit to Group 24 of Lab 2 for helping write this code

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


function motor_forces = ComputeMotorForces(Zc, Lc, Mc, Nc, R, km)
%% ComputeMotorForces
%
% Compute the individual motor forces given the required control forces
%
% Group 37
% Members:
% -Caleb Bristol
% -Adam Pillari
% -Devon Paris
% -Kushal Kedia

M = [-1 -1 -1 -1;-R/sqrt(2) -R/sqrt(2) R/sqrt(2) R/sqrt(2); ...
    R/sqrt(2) -R/sqrt(2) -R/sqrt(2) R/sqrt(2); km -km km -km];

F_c = [Zc;Lc;Mc;Nc];

motor_forces = M^(-1) * F_c;
end


function dvar_dt = linearEOM(t,var,g,m,nu,mu,Fc,Gc)
%% linearEOM
%
% Same structure as nonlinearEOM, except utilizing the linearized equations
% of motion, hence much simpler equations
%
% Group 37
% Members:
% -Caleb Bristol
% -Adam Pillari
% -Devon Paris
% -Kushal Kedia

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

%airspeed = norm(vel);
%omega_mag = norm(ang_vel);

I_B = [6.8E-5; 9.2E-5; 1.35E-4];

% Position Kinematics
dpos_dt = [vel(1);vel(2);vel(3)];

% Angular Kinematics
dang_dt = [ang_vel(1);ang_vel(2);ang_vel(3)];

% Velocity Dynamics
dvel_dt = g * [-ang(2);ang(1);0] + 1/m * [0;0;Fc(3) + m*g];

% Angular Velocity Dynamics
dangvel_dt = [Gc(1)/I_B(1);Gc(2)/I_B(2);Gc(3)/I_B(3)];

dvar_dt = [dpos_dt;dang_dt;dvel_dt;dangvel_dt];
end
