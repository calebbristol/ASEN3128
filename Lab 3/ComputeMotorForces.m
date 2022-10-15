function motor_forces = ComputeMotorForces(Zc, Lc, Mc, Nc, R, km)
%% ComputeMotorForces
%
% Group 37
% Members:
% -Caleb Bristol
% -Adam Pillari
% -Devon Paris
% -Kushal Kedia
%
% Compute the individual motor forces given the required control forces

M = [-1 -1 -1 -1;-R/sqrt(2) -R/sqrt(2) R/sqrt(2) R/sqrt(2); ...
    R/sqrt(2) -R/sqrt(2) -R/sqrt(2) R/sqrt(2); km -km km -km];

F_c = [Zc;Lc;Mc;Nc];

motor_forces = M^(-1) * F_c;
end

