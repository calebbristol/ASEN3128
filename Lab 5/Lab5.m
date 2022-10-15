clear; clc; close all;

%% Define variables
g = 9.81;
W = 2.83176*10^6; % [N]
m = W/g;
Iy = 0.449*10^8; % [kg*m^2]
theta0 = 0; % [rad]
u0 = 157.8864; % [m/s]

% Table E.3
E3 = [-4.883*10,-1.342e3,8.176e3;... 
    1.546e3,-8.561e3,-5.627e4;...
    0,-1.263e5,-1.394e7;...
    0,3.104e2,-4.138e3;...
    3.994e4,-3.341e5,-3.608e7];

% Converted to SI units
E3_SI = zeros(5,3);

for row = 1:5
    if row==1 || row==2 || row==4
        for i = 1:2
            E3_SI(row,i) = E3(row,i)*4.448/0.3048;
        end
        E3_SI(row,3) = E3(row,3)*4.448;
    end
    
    if row==3 || row==5
        for i = 1:2
            E3_SI(row,i) = E3(row,i)*4.448;
        end
        E3_SI(row,3) = E3(row,3)*4.448*0.3048;
    end
end

Xu = E3_SI(1,1); Xw = E3_SI(2,1); Xq = E3_SI(3,1); Xwdot = E3_SI(4,1); Xdelta = E3_SI(5,1);
Zu = E3_SI(1,2); Zw = E3_SI(2,2); Zq = E3_SI(3,2); Zwdot = E3_SI(4,2); Zdelta = E3_SI(5,2);
Mu = E3_SI(1,3); Mw = E3_SI(2,3); Mq = E3_SI(3,3); Mwdot = E3_SI(4,3); Mdelta = E3_SI(5,3);
zeta = deg2rad(-6.8); % [rad]

