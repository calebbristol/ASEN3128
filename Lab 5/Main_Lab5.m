%%%%%%%%%%%%%%%%%%%%%%%
%  ASEN 3128: Lab 5
%  
%  Group Members
%  -Caleb Bristol
%  -Tess Brodsky
%  -Qihan Cai
%  -Kyle Bowen
%%%%%%%%%%%%%%%%%%%%%%%

clc
clear
close all;

%% Constants
    g = 9.81; %[m/s^2]
    m = 6.366e05 * 4.448 / g; %[kg]
    u_0 = 518 * 0.3048; %[m/s]
    theta_0 = 0; %[rad]
    I_y = 3.31e07 * 14.59 * 0.3048^2; %[m/s]
    rho = 1.2673e-3 * 4.448 / 0.3048^4; %[kg/m^3]
    c = 27.31 * 0.3048; %[m]
    S = 5500 * 0.3048^2; %[m^2]

%% Part 1


    %% Problem 1
    E_3 = [-4.883*10,-1.342e3,8.176e3;1.546e3,-8.561e3,-5.627e4;0,-1.263e5,-1.394e7;0,3.104e2,-4.138e3;3.994e4,-3.341e5,-3.608e7];
    E_3_SI = E_3 .* [4.448/0.3048,4.448/0.3048,4.448;4.448/0.3048,4.448/0.3048,4.448;4.448,4.448,4.448*0.3048;4.448/0.3048,4.448/0.3048,4.448;4.448,4.448,4.448*0.3048]; 
    
    %% Problem 2
    chi = deg2rad(-6.8);
    X_u = E_3_SI(1,1)*cos(chi)^2-(E_3_SI(2,1)+E_3_SI(1,2))*sin(chi)*cos(chi)+E_3_SI(2,2)*sin(chi)^2;
    X_w = E_3_SI(2,1)*cos(chi)^2+(E_3_SI(1,1)-E_3_SI(2,2))*sin(chi)*cos(chi)-E_3_SI(1,2)*sin(chi)^2;
    X_q = E_3_SI(3,1)*cos(chi)-E_3_SI(3,2)*sin(chi);
    X_u_dot = E_3_SI(4,2)*sin(chi)^2;
    X_w_dot = -E_3_SI(4,2)*sin(chi)*cos(chi);
    Z_u = E_3_SI(1,2)*cos(chi)^2-(E_3_SI(2,2)-E_3_SI(1,1))*sin(chi)*cos(chi)-E_3_SI(2,1)*sin(chi)^2;
    Z_w = E_3_SI(2,2)*cos(chi)^2+(E_3_SI(1,2)+E_3_SI(2,1))*sin(chi)*cos(chi)+E_3_SI(1,1)*sin(chi)^2;
    Z_q = E_3_SI(3,2)*cos(chi)+E_3_SI(3,1)*sin(chi);
    Z_u_dot = -E_3_SI(4,2)*sin(chi)*cos(chi);
    Z_w_dot = E_3_SI(4,2)*cos(chi)^2;
    M_u = E_3_SI(1,3)*cos(chi) - E_3_SI(2,3)*sin(chi);
    M_w = E_3_SI(2,3)*cos(chi) + E_3_SI(1,3)*sin(chi);
    M_q = E_3_SI(3,3);
    M_u_dot = -E_3_SI(4,3)*sin(chi);
    M_w_dot = E_3_SI(4,3)*cos(chi);
    E_3_Rotated = [X_u Z_u M_u;X_w Z_w M_w;X_q Z_q M_q;X_w_dot Z_w_dot M_w_dot;X_u_dot Z_u_dot M_u_dot];
    
    % Part b
    C_w_0 = (m*g)/(.5*rho*u_0^2*S);
    
    
    Der_Table = [(X_u-rho*u_0*S*C_w_0*sin(theta_0))/(.5*rho*u_0*S) ...
        (Z_u+rho*u_0*S*C_w_0*cos(theta_0))/(.5*rho*u_0*S) ...
        M_u/(.5*rho*u_0*c*S);X_w/(.5*rho*u_0*S) ...
        Z_w/(.5*rho*u_0*S) M_w/(.5*rho*u_0*c*S);X_q/(.25*rho*u_0*c*S) ...
        Z_q/(.25*rho*u_0*c*S) M_q/(.25*rho*u_0*c^2*S); ...
        X_w_dot/(.25*rho*c*S) Z_w_dot/(.25*rho*c*S) M_w_dot/(.25*rho*c^2*S)];

    
    %% Problem 3

    
    A = [X_u/m X_w/m 0 -g*cos(theta_0);...
        Z_u/(m-Z_w_dot) Z_w/(m-Z_w_dot) (Z_q+m*u_0)/(m-Z_w_dot) (-m*g*sin(theta_0))/(m-Z_w_dot);...
        (1/I_y)*(M_u+(M_w_dot*Z_u)/(m-Z_w_dot)) (1/I_y)*(M_w+(M_w_dot*Z_w)/(m-Z_w_dot)) (1/I_y)*(M_q+(M_w_dot*(Z_q+m*u_0))/(m-Z_w_dot)) -(M_w_dot*m*g*sin(theta_0))/(I_y*(m-Z_w_dot));...
        0 0 1 0];
    
    %% Problem 4
    eigs = eig(A);
    
    % Short Period
    omega_1 = sqrt(real(eigs(1))^2 + imag(eigs(1))^2);
    zeta_1 = -real(eigs(1)) / omega_1;
    
    % Phugoid
    omega_3 = sqrt(real(eigs(3))^2 + imag(eigs(3))^2);
    zeta_3 = -real(eigs(3)) / omega_3;
    
    %% Problem 5
    
    % Short Period
    lambda_sp = M_q/(2*I_y) + 1/(2*I_y) * sqrt(M_q^2 + 4*I_y*u_0*M_w);
    
    omega_sp = sqrt(real(lambda_sp)^2 + imag(lambda_sp)^2);
    zeta_sp = -real(lambda_sp) / omega_sp;
    
    % Phugoid
    omega_ph = sqrt(2)*g/u_0;
    zeta_ph = -X_u/2 * sqrt(-u_0/(m*Z_u*g));
    P_ph = 2*pi/omega_ph;
    
    
    %% Problem 6
    
%% Part 2

    %% Problem 7
    
    % Given (Pg. 229)
    C_x_delta = -3.818e-06;
    C_z_delta = -0.3648;
    C_m_delta = -1.444;
    
    % Dimensionalize
    Xdele = C_x_delta * 0.5 * rho * u_0^2 * S;
    Zdele = C_z_delta * 0.5 * rho * u_0^2 * S;
    Mdele = C_m_delta * 0.5 * rho * u_0^2 * S * c;
    
    b_elev = [Xdele/m;Zdele/(m-Z_w_dot);Mdele/I_y + (M_w_dot*Zdele)/(I_y*(m-Z_w_dot));0];
    
    %% Problem 8
    
    % Part a)
    k_s = 1:0.01:3;
    
    b = b_elev([3 4]);
    
    A_ = @(k_s) [M_q/I_y u_0*M_w*k_s/I_y;1 0];
    
    A_nc = A_(k_s(1));
   
    % Preallocate
    eigs_sp = zeros(2,length(k_s));
    k_2 = zeros(length(k_s),1);    
    k_1 = zeros(length(k_s),1);
    
    % Calculate Control Gains
    for i = 1:length(k_s)
        A__ = A_(k_s(i));
        k_2(i) = (A_nc(1,2) - A__(1,2)) / (b(1));
        omega_ = sqrt(k_2(i)*b(1) - A__(1,2));
        k_1(i) = (2*zeta_sp*omega_ + A__(1,1)) / (b(1,1));
        eigs_sp(:,i) = eig(A_nc - b*[k_1(i) k_2(i)]);
    end
    
    %Plotting
    figure()
    plot(real(eigs_sp(1,:)),imag(eigs_sp(1,:)),'r'); hold on
    plot(real(eigs_sp(2,:)),imag(eigs_sp(2,:)),'r');
    title('Problem 8a: Complex Plane Output of k_s Values')
    xlabel('Re')
    ylabel('Im')
    xline(0)
    yline(0)
    xlim([-1 1])
    grid on; grid minor
    hold off
    
    % Part b)
    A_long_ = @(k_s) [X_u/m X_w/m 0 -g*cos(theta_0);...
        Z_u/(m-Z_w_dot) Z_w/(m-Z_w_dot) (Z_q+m*u_0)/(m-Z_w_dot) (-m*g*sin(theta_0))/(m-Z_w_dot);...
        (1/I_y)*(M_u+(M_w_dot*Z_u)/(m-Z_w_dot)) (1/I_y)*(M_w*k_s+(M_w_dot*Z_w)/(m-Z_w_dot)) (1/I_y)*(M_q+(M_w_dot*(Z_q+m*u_0))/(m-Z_w_dot)) -(M_w_dot*m*g*sin(theta_0))/(I_y*(m-Z_w_dot));...
        0 0 1 0];
    B_long = b_elev;
    
    
    eigs_8bsp = zeros(2,length(k_s));
    eigs_8bph = zeros(2,length(k_s));
    for i = 1:length(k_s)
        B_c = [[0;0;0;0] [0;0;0;0] B_long*-k_1(i) B_long*-k_2(i)];
        eigs_8b = eig(A_long_(k_s(i)) + B_c);
        eigs_8bsp(:,i) = eigs_8b([1 2]);
        eigs_8bph(:,i) = eigs_8b([3 4]);
    end
    
    %Plotting
    figure()
    plot(real(eigs_8bsp(1,:)),imag(eigs_8bsp(1,:)),'r'); hold on
    plot(real(eigs_8bsp(2,:)),imag(eigs_8bsp(2,:)),'r');
    plot(real(eigs_8bph(1,:)),imag(eigs_8bph(1,:)),'b');
    plot(real(eigs_8bph(2,:)),imag(eigs_8bph(2,:)),'b');
    scatter(real(eigs_8bsp(1,1)),imag(eigs_8bsp(1,1)),'*r');
    scatter(real(eigs_8bsp(2,1)),imag(eigs_8bsp(2,1)),'*r');
    scatter(real(eigs_8bph(1,1)),imag(eigs_8bph(1,1)),'*b');
    scatter(real(eigs_8bph(2,1)),imag(eigs_8bph(2,1)),'*b');
    title('Problem 8b: Complex Plane Output of k_s Values, Full System')
    xlabel('Re')
    ylabel('Im')
    xline(0)
    yline(0)
    xlim([-1 1])
    grid on; grid minor
    hold off
    
    % Part c)
    
    % Control Matrices & Initial Conditions
    t_span = [0 2000];
    IC_8c = [0;0;0;0.1];
    B_c = [[0;0;0;0] [0;0;0;0] B_long*-k_1(1) B_long*-k_2(1)];
    A_nc = A_long_(k_s(1)) + B_c;
    B_c = [[0;0;0;0] [0;0;0;0] B_long*-k_1(101) B_long*-k_2(101)];
    A_dc = A_long_(k_s(101)) + B_c;
    
    % Simulate Behavior
    [t_nc,x_nc] = ode45(@(t,var) linlong(t,A_nc,var),t_span,IC_8c);
    [t_dc,x_dc] = ode45(@(t,var) linlong(t,A_dc,var),t_span,IC_8c);
    
    % Plotting
    figure()
    subplot(4,1,1)
    plot(t_nc,x_nc(:,1)); hold on
    ylabel('u [m/s]')
    subplot(4,1,2)
    plot(t_nc,x_nc(:,2))
    ylabel('w [m/s]')
    subplot(4,1,3)
    plot(t_nc,x_nc(:,3))
    ylabel('q [rad/s]')
    subplot(4,1,4)
    plot(t_nc,x_nc(:,4))
    ylabel('\theta [rad]')
    xlabel('Time [s]')
    sgtitle('Problem 8c: Closed Loop Behavior, No Feedback')
    hold off
    
    figure()
    subplot(4,1,1)
    plot(t_dc,x_dc(:,1)); hold on
    ylabel('u [m/s]')
    subplot(4,1,2)
    plot(t_dc,x_dc(:,2))
    ylabel('w [m/s]')
    subplot(4,1,3)
    plot(t_dc,x_dc(:,3))
    ylabel('q [rad/s]')
    subplot(4,1,4)
    plot(t_dc,x_dc(:,4))
    ylabel('\theta [rad]')
    xlabel('Time [s]')
    sgtitle('Problem 8c: Closed Loop Behavior, Double Pitch Stiffness')
    hold off
    
    % Part d)
    
    
    % i) Translational Control Terms == 0
    
    
    % Control Matrices & Initial Conditions
    IC_8d = [0;0;0;0.1];
    
    B_d1 = [0/m;0/(m-0);Mdele/I_y + (M_w_dot*0)/(I_y*(m-0));0];
    B_d = [[0;0;0;0] [0;0;0;0] B_d1*-k_1(1) B_d1*-k_2(1)];
    A_d1 = A_long_(k_s(1)) + B_d;
    
    % Simulate Behavior
    [t_d1,x_d1] = ode45(@(t,var) linlong(t,A_d1,var),t_span,IC_8d);
    
    % Plotting
    figure()
    subplot(4,1,1)
    plot(t_d1,x_d1(:,1)); hold on
    ylabel('u [m/s]')
    subplot(4,1,2)
    plot(t_d1,x_d1(:,2))
    ylabel('w [m/s]')
    subplot(4,1,3)
    plot(t_d1,x_d1(:,3))
    ylabel('q [rad/s]')
    subplot(4,1,4)
    plot(t_d1,x_d1(:,4))
    ylabel('\theta [rad]')
    xlabel('Time [s]')
    sgtitle('Problem 8d,i: Closed Loop Behavior, Translational Control = 0')
    hold off
   
    
    % ii) Rotational Terms out of Translational EOM and Vice-Versa
    
    
    % Control Matrices & Initial Conditions
    IC_8d = [0;0;0;0.1];
    
    B_d = [[0;0;0;0] [0;0;0;0] B_long*-k_1(1) B_long*-k_2(1)];
    A_long_d2 = [X_u/m X_w/m 0 -g*cos(theta_0);...
        Z_u/(m-Z_w_dot) Z_w/(m-Z_w_dot) (0+m*u_0)/(m-0) (-m*g*sin(theta_0))/(m-0);...
        (1/I_y)*(0+(0*Z_u)/(m-Z_w_dot)) (1/I_y)*(0+(0*Z_w)/(m-Z_w_dot)) (1/I_y)*(M_q+(M_w_dot*(0+m*u_0))/(m-0)) -(M_w_dot*m*g*sin(theta_0))/(I_y*(m-0));...
        0 0 1 0];
    A_d2 = A_long_(k_s(1)) + B_d;
    
    % Simulate Behavior
    [t_d2,x_d2] = ode45(@(t,var) linlong(t,A_d2,var),t_span,IC_8d);
    
    % Plotting
    figure()
    subplot(4,1,1)
    plot(t_d2,x_d2(:,1)); hold on
    ylabel('u [m/s]')
    subplot(4,1,2)
    plot(t_d2,x_d2(:,2))
    ylabel('w [m/s]')
    subplot(4,1,3)
    plot(t_d2,x_d2(:,3))
    ylabel('q [rad/s]')
    subplot(4,1,4)
    plot(t_d2,x_d2(:,4))
    ylabel('\theta [rad]')
    xlabel('Time [s]')
    sgtitle('Problem 8d,ii: Closed Loop Behavior, Translational/Rotational Separated')
    hold off
    
    
    % iii) System Decoupled
    
    
    % Control Matrices & Initial Conditions
    IC_8d = [0;0;0;0.1];
    
    A_d3 = A_long_(k_s(1)).*eye(4) + B_d;
    
    % Simulate Behavior
    [t_d3,x_d3] = ode45(@(t,var) linlong(t,A_d3,var),t_span,IC_8d);
    
    % Plotting
    figure()
    subplot(4,1,1)
    plot(t_d3,x_d3(:,1)); hold on
    ylabel('u [m/s]')
    subplot(4,1,2)
    plot(t_d3,x_d3(:,2))
    ylabel('w [m/s]')
    subplot(4,1,3)
    plot(t_d3,x_d3(:,3))
    ylabel('q [rad/s]')
    subplot(4,1,4)
    plot(t_d3,x_d3(:,4))
    ylabel('\theta [rad]')
    xlabel('Time [s]')
    sgtitle('Problem 8d,iii: Closed Loop Behavior, Decoupled')
    hold off
    
    
%% Functions

function dxdt = linlong(t,A,x_0)
    dxdt = A*x_0;
end
    