%%%%%%%%%%%%%%%%%%%%%%%%%
%  ASEN 3128 Lab 6
%  Author: Me
%  Date: Now
%
%%%%%%%%%%%%%%%%%%%%%%%%%


%% 
clc
clear
close all;


%% Constants
g = 9.81; %[m/s^2]


%% Part 1: Lateral Dynamics Modeling

    
    %% Problem 1
    %
    %
    
    % Given
    C_y_beta = -0.8771;
    C_l_beta = -0.2797;
    C_n_beta = 0.1946;
    C_y_p = 0;
    C_l_p = -0.3295;
    C_n_p = -0.04073;
    C_y_r = 0;
    C_l_r = 0.304;
    C_n_r = -0.2737;
    h = 20000 * 0.3048; %[ft -> m]
    M = 0.5;
    S = 5500 * 0.3048^2; %[ft^2 -> m^2]
    b = 195.68 * 0.3048; %[ft -> m]
    c = 27.31 * 0.3048; %[ft -> m]
    V = 518 * 0.3048; %[ft/s -> m/s]
    W = 6.366e05 * 4.448; %[lb -> N]
    m = W/g; %[kg]
    I_x = 1.82e07 * 14.59 * 0.3048^2; %[slug*ft^2 -> kg*m^2]
    I_y = 3.31e07 * 14.59 * 0.3048^2; %[slug*ft^2 -> kg*m^2]
    I_z = 4.97e07 * 14.59 * 0.3048^2; %[slug*ft^2 -> kg*m^2]
    I_zx = 9.70e05 * 14.59 * 0.3048^2; %[slug*ft^2 -> kg*m^2]
    zeta = -6.8; %[deg]
    C_D = 0.040;
    
    %Construct Table 6.7
    
    
    
%% Part 2: Feedback Control for Stability Augmentation
%
%

    %% Problem 6
    %
    %
    
    %% Ranges
    Del_del_a_a = 0:0.01:10;
    Del_del_r_a = 0;
    Del_del_a_b = 0:-0.01:-10;
    Del_del_r_b = 0;
    Del_del_a_c = 0;
    Del_del_r_c = 0:0.01:5;
    Del_del_a_d = 0;
    Del_del_r_d = 0:0.01:5;
    
    A_aug = load('A_aug.mat');
    A_lat_aug = A_aug.A_aug;
    B_aug = load('B_aug.mat');
    B_lat_aug = B_aug.B_aug;
    
    %% Construct Full Matrix
    
    % a)
    A_a = @(ind) A_lat_aug + B_lat_aug * [0 0 0 Del_del_a_a(ind) 0 0;0 0 0 Del_del_r_a 0 0];
    
    % b)
    A_b = @(ind) A_lat_aug + B_lat_aug * [0 Del_del_a_b(ind) 0 0 0 0;0 Del_del_r_b 0 0 0 0];
    
    % c)
    A_c = @(ind) A_lat_aug + B_lat_aug * [0 0 Del_del_a_c 0 0 0;0 0 Del_del_r_c(ind) 0 0 0];
    A_c_neg = @(ind) A_lat_aug + B_lat_aug * [0 0 Del_del_a_c 0 0 0;0 0 -Del_del_r_c(ind) 0 0 0];
    
    % d)
    A_d = @(ind) A_lat_aug + B_lat_aug * [0 0 0 0 Del_del_a_d 0;0 0 0 0 Del_del_r_d(ind) 0];
    A_d_neg = @(ind) A_lat_aug + B_lat_aug * [0 0 0 0 Del_del_a_d 0;0 0 0 0 -Del_del_r_d(ind) 0];
    
    % Combined
    A_comb = @(ind1,ind2) A_lat_aug + B_lat_aug * [0 0 0 Del_del_a_a(ind1) 0 0;0 0 Del_del_r_c(ind2) 0 0 0];
    
    
    %% Determine Eigenvalues
    Eig_a = zeros(6,length(Del_del_a_a));
    Eig_b = zeros(6,length(Del_del_a_b));
    Eig_c_pos = zeros(6,length(Del_del_r_c));
    Eig_c_neg = zeros(6,length(Del_del_r_c));
    Eig_d_pos = zeros(6,length(Del_del_r_d));
    Eig_d_neg = zeros(6,length(Del_del_r_d));
    
    % a)
    for i = 1:length(Del_del_a_a)
        Eig_a(:,i) = eig(A_a(i));
    end
    
    % b)
    for i = 1:length(Del_del_a_b)
        Eig_b(:,i) = eig(A_b(i));
    end
    
    % c)
    for i = 1:length(Del_del_r_c)
        Eig_c_pos(:,i) = eig(A_c(i));
    end
    
    for i = 1:length(Del_del_r_c)
        Eig_c_neg(:,i) = eig(A_c_neg(i));
    end
    
    % d)
    for i = 1:length(Del_del_r_d)
        Eig_d_pos(:,i) = eig(A_d(i));
    end
    
    for i = 1:length(Del_del_r_d)
        Eig_d_neg(:,i) = eig(A_d_neg(i));
    end
    
    % Combined Loops
    
    %{
    Eig_comb = zeros(6,length(length(Del_del_a_a))*length(Del_del_r_c));
    colorgrad_a_a = zeros(length(length(Del_del_a_a))*length(Del_del_r_c),1);
    colorgrad_r_c = zeros(length(length(Del_del_a_a))*length(Del_del_r_c),1);
    for i = 1:length(Del_del_a_a)
        for j = 1:length(Del_del_r_c)
            Eig_comb(:,length(Del_del_a_a)*(i-1)+j) = eig(A_comb(i,j));
            colorgrad_a_a(length(Del_del_a_a)*(i-1)+j) = Del_del_a_a(i);
            colorgrad_r_c(length(Del_del_a_a)*(i-1)+j) = Del_del_r_c(j);
        end
    end
    %}
    
    
    
    %% Plot Eigenvalue Locus Plots
    
    
    x__ = -1:0.01:1;
    y__ = tand(69.5)*x__;
    
    % a)
    figure()
    for i = 1:6
        scatter(real(Eig_a(i,:)),imag(Eig_a(i,:)),20,Del_del_a_a); hold on     
    end
    colorbar()
    plot(x__,y__)
    plot(x__,-y__)
    title('Problem 6a: Eigenvalue Locus of \phi to a Controls')
    xlabel('Re')
    ylabel('Im')
    xline(-0.04)
    xline(-0.025)
    grid on; grid minor;
    hold off
    
    % b)
    figure()
    for i = 1:6
        scatter(real(Eig_b(i,:)),imag(Eig_b(i,:)),20,Del_del_a_b); hold on     
    end
    colorbar()
    plot(x__,y__)
    plot(x__,-y__)
    title('Problem 6b: Eigenvalue Locus of p to a Controls')
    xlabel('Re')
    ylabel('Im')
    xline(-0.04)
    xline(-0.025)
    grid on; grid minor;
    hold off
    
    % c)
    figure()
    for i = 1:6
        scatter(real(Eig_c_pos(i,:)),imag(Eig_c_pos(i,:)),20,Del_del_r_c); hold on     
    end
    colorbar()
    plot(x__,y__)
    plot(x__,-y__)
    title('Problem 6c: Eigenvalue Locus of r to a Controls')
    xlabel('Re')
    ylabel('Im')
    xline(-0.04)
    xline(-0.025)
    grid on; grid minor;
    hold off
    
    figure()
    for i = 1:6
        scatter(real(Eig_c_neg(i,:)),imag(Eig_c_neg(i,:)),20,-Del_del_r_c); hold on     
    end
    colorbar()
    plot(x__,y__)
    plot(x__,-y__)
    title('Problem 6c: Eigenvalue Locus of r to a Controls')
    xlabel('Re')
    ylabel('Im')
    xline(-0.04)
    xline(-0.025)
    grid on; grid minor;
    hold off
    
    % d)
    figure()
    for i = 1:6
        scatter(real(Eig_d_pos(i,:)),imag(Eig_d_pos(i,:)),20,Del_del_r_d); hold on     
    end
    colorbar()
    plot(x__,y__)
    plot(x__,-y__)
    title('Problem 6d: Eigenvalue Locus of \psi to a Controls')
    xlabel('Re')
    ylabel('Im')
    xline(-0.04)
    xline(-0.025)
    grid on; grid minor;
    hold off
    
    figure()
    for i = 1:6
        scatter(real(Eig_d_neg(i,:)),imag(Eig_d_neg(i,:)),20,-Del_del_r_d); hold on     
    end
    colorbar()
    plot(x__,y__)
    plot(x__,-y__)
    title('Problem 6d: Eigenvalue Locus of \psi to a Controls')
    xlabel('Re')
    ylabel('Im')
    xline(-0.04)
    xline(-0.025)
    grid on; grid minor;
    hold off
    
    % Combined
    %{
    figure()
    for i = 1:6
        scatter(real(Eig_comb(i,:)),imag(Eig_comb(i,:)),20,colorgrad_a_a); hold on
    end
    colorbar()
    plot(x__,y__)
    plot(x__,-y__)
    title('Problem 6 Combined: Eigenvalue Locus')
    xlabel('Re')
    ylabel('Im')
    xline(-0.04)
    xline(-0.025)
    grid on; grid minor;
    hold off
    
    figure()
    for i = 1:6
        scatter(real(Eig_comb(i,:)),imag(Eig_comb(i,:)),20,colorgrad_r_c); hold on
    end
    colorbar()
    plot(x__,y__)
    plot(x__,-y__)
    title('Problem 6 Combined: Eigenvalue Locus')
    xlabel('Re')
    ylabel('Im')
    xline(-0.04)
    xline(-0.025)
    grid on; grid minor;
    hold off
    %}
    
    
%% Problem 7
%
%

    %% Simulate ODE45
    tspan = [0 1000];
    IC_a = [10;-0.14;0.05;0;0;0];
    
    k_test = [0 0 0 4 0 0;0 0 0 0 0.1 0];
    
    [t_sim,x_sim] = ode45(@(t,var) shigunghwa(t,var,A_lat_aug,B_lat_aug,k_test),tspan,IC_a);
    
    
    %% Plotting
    figure()
    subplot(6,1,1)
    plot(t_sim,x_sim(:,1)); hold on
    subplot(6,1,2)
    plot(t_sim,x_sim(:,2))
    subplot(6,1,3)
    plot(t_sim,x_sim(:,3))
    subplot(6,1,4)
    plot(t_sim,x_sim(:,4))
    subplot(6,1,5)
    plot(t_sim,x_sim(:,5))
    subplot(6,1,6)
    plot(t_sim,x_sim(:,6))
    
    
    
%% Functions

function dxdt = shigunghwa(t,x,A,B,k)
    dxdt = (A + B*k)*x;
end
    