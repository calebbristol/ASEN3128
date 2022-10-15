%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  ASEN 3128 HW 3
%  
%  Author: Caleb Bristol
%  Due Date: 24 February, 2022
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%% Clean Your Workstation
clc
clear


%% Problem 1


    %% Constants (Vector and Individual Form)
    
    % State Vector
    x_ = [100;100;-1600;0.17;0.17;2.4;1;-1;0;0;0;0];
    x = x_(1);
    y = x_(2);
    z = x_(3);
    phi = x_(4);
    theta = x_(5);
    psi = x_(6);
    u = x_(7);
    v = x_(8);
    w = x_(9);
    p = x_(10);
    q = x_(11);
    r = x_(12);
    
    % State Derivative Vector
    x_dot_ = [-0.04;1.37;-0.34;0;0;0;-1.68;1.66;-0.3;-20.79;-15.37;0];
    
    % As pulled from the lab document:
    %
    % For Parrot Mamba Minidrone
    g = 9.81;       %[m/s^2]
    m = 0.068;      %[kg]
    R = 0.060;      %[m]
    k_m = 0.0024;   %[Nm/N]
    I_x = 6.8E-5;   %[kg*m^2]
    I_y = 9.2E-5;   %[kg*m^2]
    I_z = 1.35E-4;  %[kg*m^2]
    nu = 1E-3;      %[N/(m/s)^2]
    mu = 2E-6;      %[Nm/(rad/s)^2]
    
    
    %% Determine Forces and Moments
    %
    % The methodology for this section was to calculate the aerodynamic
    % forces and moments first, based off the velocity and angular velocity
    % of the craft, then utilize equations of motion to extract the control
    % forces and moments second.
    
    
        %% Aerodynamic Forces and Moments
        
        % Aerodynamic Drag Force
        airspeed = norm(x_(7:9));
        
        D = nu * airspeed^2;
        
        f_aero = -D * x_(7:9) / norm(x_(7:9));
        
        % Aerodynamic Moments
        m_aero = -mu * norm(x_(10:12)) * x_(10:12);
        
        %% Control Forces and Moments
        
        % Control Forces
        f_c = m*(x_dot_(7:9)) - m*([r*v-q*w;p*w-r*u;q*u-p*v] + g*[-sin(theta);cos(theta)*sin(phi);cos(theta)*cos(phi)]) - f_aero;
    
        % Control Moments
        I_B = [I_x 0 0;0 I_y 0;0 0 I_z];
        
        m_c = I_B*(x_dot_(10:12) - [(I_y-I_z)*q*r/I_x;(I_z-I_x)*r*p/I_y;(I_x-I_y)*p*q/I_z] - I_B^(-1)*m_aero);
    
    %% Determine Motor Thrust Forces
    
        % Done Utilizing Function from Lab
        f_ = ComputeMotorForces(f_c(3),m_c(1),m_c(2),m_c(3),R,k_m);
        
     
    %% Display Results
    fprintf('PROBLEM 1: \n \n')
    fprintf('The aerodynamic forces acting on the craft [N] were: \n')
    disp(f_aero)
    fprintf('The aerodynamic moments acting on the craft [Nm] were: \n')
    disp(m_aero)
    fprintf('The control forces acting on the craft [N] were: \n')
    disp(f_c)
    fprintf('The control moments acting on the craft [Nm] were: \n')
    disp(m_c)
    fprintf('The force exerted by motors 1 through 4 [N] were: \n')
    disp(f_)
    
        
%% Problem 2
%
% TRUE

    %% Explanation
    %
    % Considering that the quadrotor was in steady hover, the control
    % forces applied should be just enough that the aircraft experiences
    % thrust equal to its weight in the z-direction, exactly enough to
    % counteract the force of gravity. Now consider the addition of some
    % delta theta, or instantaneous pitch. The force of gravity will still
    % act in the downwards direction with equal magnitude, but if the
    % forces applied by the motors don't change, then the force applied by
    % the motors will no longer be opposing the force of gravity, because
    % the aircraft will be pitched. It will begin to translate in the x
    % direction but more importantly will begin to descend. Because the
    % force applied by the motors opposing the direction of gravity will no
    % longer be equal to gravity g, but g times the cosine of delta theta,
    % it will not be enough to counteract gravity unless theta is zero,
    % which according to our instantaneous pitch added, it is not.
    % Therefore, the force applied by the motors will be slightly less than
    % that of gravity and the aircraft will slowly begin to descend in the
    % downwards direction, and descend. 
    %
    % Therefore, the statement is TRUE.
    
    %% Display Results?
    fprintf('PROBLEM 2: \n \n')
    fprintf('TRUE: see commented code for detailed explanation. \n')

    
%% Functions
%
% This function was pulled from Lab 3


function motor_forces = ComputeMotorForces(Zc, Lc, Mc, Nc, R, km)
%% ComputeMotorForces
%
% Compute the individual motor forces given the required control forces

M = [-1 -1 -1 -1;-R/sqrt(2) -R/sqrt(2) R/sqrt(2) R/sqrt(2); ...
    R/sqrt(2) -R/sqrt(2) -R/sqrt(2) R/sqrt(2); km -km km -km];

F_c = [Zc;Lc;Mc;Nc];

motor_forces = M^(-1) * F_c;
end
