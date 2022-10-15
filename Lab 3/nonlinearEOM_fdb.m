function dvar_dt = nonlinearEOM_fdb(t,var,g,m,nu,mu,Fc,Gc)
%% nonlinearEOM_fdb
%
% Group 37
% Members:
% -Caleb Bristol
% -Adam Pillari
% -Devon Paris
% -Kushal Kedia
%
% Nonlinear equations of motion, including feedback control as specified in
% problem 5

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
