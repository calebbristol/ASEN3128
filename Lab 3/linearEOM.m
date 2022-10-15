function dvar_dt = linearEOM(t,var,g,m,nu,mu,Fc,Gc)
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

