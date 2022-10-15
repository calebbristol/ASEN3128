%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ASEN 3128: Lab 4
%
% Group Members:
% -Caleb Bristol
% -Devon Paris
% -Adam Pillari
% -Kushal Kedia
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Clear Workspace
clear
clc
close all;

%% Constants
g = 9.81;       %[m/s^2]
m = 0.068;      %[kg]
r = 0.060;      %[m]
k_m = 0.0024;   %[Nm/N]
I_x = 5.8E-5;   %[kg*m^2]
I_y = 7.2E-5;   %[kg*m^2]
I_z = 1.0E-4;  %[kg*m^2]
nu = 1E-3;      %[N/(m/s)^2]
mu = 2E-6;      %[Nm/(rad/s)^2]


%% Problem 1
%
% This problem outlines a methodology to determine proper control gains for
% the roll and roll rate from the linearized equations of motion, to
% (hopefully) create a stable system



    
%     %% Solve For Control Gains
%     %
%     % 
%     tau_1 = 0.5; %[s]
%     sigma_1 = -1/tau_1;
%     
%         %% Longitudinal
%         k_1_long_ = 0:1e-05:0.01;
%         k_2_long_ = 0:1e-05:0.01;
%         flag_long = zeros(length(k_1_long_),length(k_2_long_));
%         for i = 1:length(k_1_long_)
%             for j = 1:length(k_2_long_)
%                 A = [0 1;-k_2_long_(j)/I_y -k_1_long_(i)/I_y];
%                 eig_A = eig(A);
%                 if real(eig_A) < 0
%                     if isreal(eig_A) && abs((max(eig_A)+2)) < 0.01
%                         if eig_A(1) ~= eig_A(2)
%                             flag_long(i,j) = 1;
%                         end
%                     end
%                 end
%             end
%         end
%         
%         k_1_long = [];
%         k_2_long = [];
%         for i = 1:length(k_1_long_)
%             for j = 1:length(k_2_long_)
%                 if flag_long(i,j) == 1
%                     k_1_long = [k_1_long k_1_long_(i)];
%                     k_2_long = [k_2_long k_2_long_(j)];
%                 end
%             end
%         end
%         
%         
%         
%         
%         %% Lateral
%         
%         k_1_lat_ = 0:1e-06:0.01;
%         k_2_lat_ = 0:1e-06:0.01;
%         flag_lat = zeros(length(k_1_lat_),length(k_2_lat_));
%         for i = 1:length(k_1_lat_)
%             for j = 1:length(k_2_lat_)
%                 A = [0 1;-k_2_lat_(j)/I_x -k_1_lat_(i)/I_x];
%                 eig_A = eig(A);
%                 if real(eig_A) < 0
%                     if isreal(eig_A) && abs((max(eig_A)+2)) < 0.01
%                         if eig_A(1) ~= eig_A(2)
%                             flag_lat(i,j) = 1;
%                         end
%                     end
%                 end
%             end
%         end
%         
%         k_1_lat = [];
%         k_2_lat = [];
%         for i = 1:length(k_1_lat_)
%             for j = 1:length(k_2_lat_)
%                 if flag_lat(i,j) == 1
%                     k_1_lat = [k_1_lat k_1_lat_(i)];
%                     k_2_lat = [k_2_lat k_2_lat_(j)];
%                 end
%             end
%         end
        
   
%% Problem 2
%
% Simulate a closed loop linearized system response over a 10 second window


    %% Establish Problem Constraints
    
%     % Initial Conditions
%     IC_a = [0; 0; 0; deg2rad(5); 0; 0; 0; 0; 0; 0; 0; 0];
%     IC_b = [0; 0; 0; 0; deg2rad(5); 0; 0; 0; 0; 0; 0; 0];
%     IC_c = [0; 0; 0; 0; 0; 0; 0; 0; 0; 0.1; 0; 0];
%     IC_d = [0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0.1; 0];
%     
%     % Time span
%     tspan = [0 10];%[s]
%     
%     % Solved by using the w_dot quadrotor equation and omitting aerodynamic forces
%     Zc_1 = -m*g;
% 
%     % Solved by using systems of equations on control forces/moments with Lc,
%     % Mc, and Nc = 0
%     f1 = m*g/4;     %[N]
%     f2 = m*g/4;
%     f3 = m*g/4;
%     f4 = m*g/4;
%     
%     % Setting up control forces/moments vectors
%     Fc_1 = [0; 0; Zc_1];
%     Gc_1 = [(r/sqrt(2))*(-f1-f2+f3+f4); (r/sqrt(2))*(f1-f2-f3+f4); k_m*(f1-f2+f3-f4)];
%     k_vec = [k_1_long(1);k_2_long(1);0;k_1_lat(1);k_2_lat(1);0];
% 
%     
%     %% Run Simulation
%     [t_out_a,var_out_a] = ode45(@(t,var) linearEOM_ctrl(t,var,g,m,nu,mu,Fc_1,Gc_1,k_vec),tspan,IC_a);
%     [t_out_b,var_out_b] = ode45(@(t,var) linearEOM_ctrl(t,var,g,m,nu,mu,Fc_1,Gc_1,k_vec),tspan,IC_b);
%     [t_out_c,var_out_c] = ode45(@(t,var) linearEOM_ctrl(t,var,g,m,nu,mu,Fc_1,Gc_1,k_vec),tspan,IC_c);
%     [t_out_d,var_out_d] = ode45(@(t,var) linearEOM_ctrl(t,var,g,m,nu,mu,Fc_1,Gc_1,k_vec),tspan,IC_d);
%     
%     
%     %% Plotting
%     f_c_a = [f1*ones(length(t_out_a),1) f2*ones(length(t_out_a),1) f3*ones(length(t_out_a),1) f4*ones(length(t_out_a),1)];
%     f_c_b = [f1*ones(length(t_out_b),1) f2*ones(length(t_out_b),1) f3*ones(length(t_out_b),1) f4*ones(length(t_out_b),1)];
%     f_c_c = [f1*ones(length(t_out_c),1) f2*ones(length(t_out_c),1) f3*ones(length(t_out_c),1) f4*ones(length(t_out_c),1)];
%     f_c_d = [f1*ones(length(t_out_d),1) f2*ones(length(t_out_d),1) f3*ones(length(t_out_d),1) f4*ones(length(t_out_d),1)];
%     
%     %PlotAircraftSim(t_out_a, var_out_a',f_c_a', 1:6, ["-r","a";"-","b";"-","c";"-","d"])
%     %PlotAircraftSim(t_out_b, var_out_b',f_c_b', 1:6, ["-b","a";"-","b";"-","c";"-","d"])
%     %PlotAircraftSim(t_out_c, var_out_c',f_c_c', 1:6, ["-g","a";"-","b";"-","c";"-","d"])
%     %PlotAircraftSim(t_out_d, var_out_d',f_c_d', 1:6, ["-k","a";"-","b";"-","c";"-","d"])
    
    
%% Problem 3


%% Problem 4
%
% This problem involves finding a k_3 value such that our system will have
% real eigenvalues, and a time constant less than 1.25 s. 

%     %% Longitudnal
%     k_3_long_ = -0.001:1e-06:0.001;
%     flag_k_3_long = zeros(length(k_3_long_),length(k_1_long));
%     for i = 1:length(k_3_long_)
%         for j = 1:length(k_1_long)
%             A = [0 1 0 0;0 0 -g 0;0 0 0 1;0 -k_3_long_(i)/I_y -k_2_long(j)/I_y -k_1_long(j)/I_y];
%             eig_A = eig(A);
%             eig_long(:,i) = eig_A;
%             if isreal(eig_A) && max(eig_A) < 0.8
%                 flag_k_3_long(i,j) = 1;
%             end
%         end
%     end
%     
%     k_3_long = 0;
%     for i = 1:length(k_3_long_)
%         for j = 1:length(k_1_long)
%             if flag_k_3_long(i,j) == 1 && k_3_long == 0
%                 k_3_long = k_3_long_(i);
%                 k_1_long = k_1_long(j);
%                 k_2_long = k_2_long(j);
%             end
%         end
%     end
%     %% Lateral
%     k_3_lat_ = -0.0001:1e-06:0.0001;
%     flag_k_3_lat = zeros(length(k_3_lat_),length(k_1_lat));
%     for i = 1:length(k_3_lat_)
%         for j = 1:length(k_1_lat)
%             A = [0 1 0 0;0 0 g 0;0 0 0 1;0 -k_3_lat_(i)/I_x -k_2_lat(j)/I_x -k_1_lat(j)/I_x];
%             eig_A = eig(A);
%             eig_lat(:,i) = eig_A;
%             if isreal(eig_A) && min(-1./eig_A(2:4)) < 1.25
%                 if real(eig_A(2:4)) < 0
%                     flag_k_3_lat(i,j) = 1;
%                 end
%             end
%         end
%     end
%     
%     k_3_lat = 0;
%     for i = 1:length(k_3_lat_)
%         for j = 1:length(k_1_lat)
%             if flag_k_3_lat(i,j) == 1 && k_3_lat == 0
%                 k_3_lat = k_3_lat_(i);
%                 k_1_lat = k_1_lat(j);
%                 k_2_lat = k_2_lat(j);
%             end
%         end
%     end
    
    %% Generate Eigenvalue Plots
    k1y =5.0400e-04;
    k2y =7.2000e-04;

    k3y=-0.4*10^-4;
    k4y=1;
    k_3_long = -1e-03:1e-05:0;
    for i = 1:length(k_3_long)
        Along=[0 1 0 0;
            0 0 g 0;
            0 0 0 1;
             -k_3_long(i)*k4y/I_y,-k_3_long(i)/I_y,-k2y/I_y,-k1y/I_y];
         eig_long(:,i) = eig(Along);
    end
    Along=[0 1 0 0;
        0 0 g 0;
        0 0 0 1;
         -k3y*k4y/I_y,-k3y/I_y,-k2y/I_y,-k1y/I_y];
     eiglong = eig(Along);
    
    k1x = 4.0600e-04;
    k2x =5.8000e-04;

    k4x=1;
    k3x=.4*10^-4;
    k_3_lat = 0:1e-05:1e-03;
    for i = 1:length(k_3_lat)
        Alat=[0 1 0 0;
            0 0 -g 0;
            0 0 0 1;
             -k_3_lat(i)*k4x/I_x,-k_3_lat(i)/I_x,-k2x/I_x,-k1x/I_x];
         eig_lat(:,i) = eig(Alat);
    end
    A_lat=[0 1 0 0;
        0 0 -g 0;
        0 0 0 1;
         -k3x*k4x/I_x,-k3x/I_x,-k2x/I_x,-k1x/I_x];
     eiglat = eig(A_lat);

    %% Plotting
    figure()
    plot(real(eig_long(1,:)),imag(eig_long(1,:)),'b'); hold on
    plot(real(eig_long(2,:)),imag(eig_long(2,:)),'r');
    plot(real(eig_long(3,:)),imag(eig_long(3,:)),'k');
    plot(real(eig_long(4,:)),imag(eig_long(4,:)),'m');
    scatter(eiglong(1),0,'*b');
    scatter(eiglong(2),0,'*r');
    scatter(eiglong(3),0,'*k');
    scatter(eiglong(4),0,'*m');
    legend('\lambda_1','\lambda_2','\lambda_3','\lambda_4')
    xlabel('Re')
    ylabel('Im')
    set(gca,'fontsize',11)
    title('Eigenvalues For Varying k_3 Values: Longitudinal')
    grid on; grid minor
    hold off
    
    figure()
    plot(real(eig_lat(1,:)),imag(eig_lat(1,:)),'b'); hold on
    plot(real(eig_lat(2,:)),imag(eig_lat(2,:)),'r');
    plot(real(eig_lat(3,:)),imag(eig_lat(3,:)),'k');
    plot(real(eig_lat(4,:)),imag(eig_lat(4,:)),'m');
    scatter(eiglat(1),0,'*b');
    scatter(eiglat(2),0,'*r');
    scatter(eiglat(3),0,'*k');
    scatter(eiglat(4),0,'*m');
    legend('\lambda_1','\lambda_2','\lambda_3','\lambda_4')
    xlabel('Re')
    ylabel('Im')
    set(gca,'fontsize',11)
    title('Eigenvalues For Varying k_3 Values: Lateral')
    grid on; grid minor
    hold off


%% Problem 5

        

%% Functions

function dvar_dt = linearEOM_ctrl(t,var,g,m,nu,mu,Fc,Gc,k_vec)
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

k_1_long = k_vec(1);
k_2_long = k_vec(2);
k_3_long = k_vec(3);

k_1_lat = k_vec(4);
k_2_lat = k_vec(5);
k_3_lat = k_vec(6);



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
