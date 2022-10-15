function [] = PlotAircraftSim(time, aircraft_state_array, control_input_array, fig, col)
%% PlotAircraftSim
%
% Group 37
% Members:
% -Caleb Bristol
% -Adam Pillari
% -Devon Paris
% -Kushal Kedia
%
% Function to plot all aircraft variables over time given the state vector,
% control inputs vector, and a column of plotting options

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

