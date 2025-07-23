% my drone model

clc;
clear all;
close all;

q_m= 0.033 ; % drone mass (kg)
l= 39.73e-3 ; % arm length of the drone (m)
g= 9.81; % gravity (m/s^2)
k_f = 2.8799e-08;
drone_configuration = 'cross'; % drone configuration 'plus' or 'cross'
trajectory = 'spiral'; % trajectory type 'hoverZ', 'spiral' , 'hoverX', 'hoverY', 'hoverXY'.

gains = load('tunedGains.mat');
gains = gains.Kopt % load the tuned gains
gains = struct('kp_x',gains(1),'kd_x',gains(2),'ki_x',gains(3), ...
               'kp_y',gains(4),'kd_y',gains(5),'ki_y',gains(6))
               
hard_tilt = 0; % hard tilt mode

t_span = [0 50];

%% state equations
x_i = zeros(18,1);
x_i(1) = 0; % x position
x_i(3) = 0; % y position

% Hover speed (rad/s) for Crazyflie at sea‑level
omega_hover = sqrt((q_m * g) / (4*k_f));
x_i(13:16)  = omega_hover;

[t, x] = ode45 (@(t, x) drone_controller( q_m, l, g, k_f, hard_tilt, gains, trajectory, drone_configuration, t, x), t_span, x_i, odeset('RelTol',1e-3,'AbsTol',1e-4));
x=x';
%% simulations

% Trajectory

% tra_x = 2*sin(0.5*t);
% tra_y = 2*cos(0.5*t);
% tra_z = 0.1*t;
% tra = [tra_x tra_y tra_z]';
tra = zeros(3, length(t));
tra_d = zeros(3, length(t));
for i = 1:length(t)
    [tra_p, tra_v] = ref_trajectory(t(i), trajectory);
    tra(:, i) = tra_p;
    tra_d(:, i) = tra_v;
end


figure(1);
plot(t,x(1,:),' r--')
hold on 
plot(t,x(3,:),' b--')
hold on 
plot(t,x(5,:),' k--')
title('Position of the Quadcoper');
xlabel('time(s) ')
ylabel(' meter ')
legend('Position x', 'Position y', 'Altitute')
saveas(gcf, 'drone_position.png');


figure (2)
plot3(x(1,:), x(3,:), x(5,:))
title ('Trajectory of the drone')
xlabel ('Position x')
ylabel ('Position y')
zlabel ('Altitute')
grid on
saveas(gcf, 'drone_trajectory.png');

figure (3)
plot3(x(1,:), x(3,:), x(5,:))
hold on
plot3(tra(1,:), tra(2,:), tra(3,:), 'r-', 'LineWidth', 2)
% plot3(sin(k_1*t), cos(k_2*t) ,z_d*ones(length(t)), 'r')
xlabel('Position x')
ylabel(' Position y ')
zlabel ('Altitude')
grid on
% legend('Path of the drone UAV', 'Desired Trajectory ')
title('drone Path vs Desired Trajectory')
saveas(gcf, 'drone_path_vs_trajectory.png');

figure (4)
subplot(3,1,1)
plot(t, tra(1,:), 'r-', 'LineWidth', 2)
hold on
plot(t, x(1,:), 'b-', 'LineWidth', 2)
title('Position error on x')
xlabel('time (s)')
ylabel('position error (m)')
subplot(3,1,2)
plot(t, tra(2,:), 'r-', 'LineWidth', 2)
hold on
plot(t, x(3,:), 'b-', 'LineWidth', 2)
title('Position error on y')
xlabel('time (s)')
ylabel('position error (m)')
subplot(3,1,3)
plot(t, tra(3,:), 'r-', 'LineWidth', 2)
hold on
plot(t, x(5,:), 'b-', 'LineWidth', 2)
title('Position error on z')
xlabel('time (s)')
ylabel('position error (m)')
grid on
saveas(gcf, 'drone_position_error.png');

figure(5);
subplot(3,1,1)
plot(t,x(7,:),' r')
title('attitude, phi');
xlabel('time(s) ')
ylabel('angle')
subplot(3,1,2)
plot(t,x(9,:),' r')
title('attitude, theta');
xlabel('time(s) ')
ylabel('angle')
subplot(3,1,3)
plot(t,x(11,:),' r')
title('attitude, psi');
xlabel('time(s) ')
ylabel('angle')
grid on

figure(8);
subplot(3,1,1)
plot(t, tra(1,:) - x(1,:), 'r-', 'LineWidth', 2)
title('Position error on x')
xlabel('time (s)')
ylabel('position error (m)')
subplot(3,1,2)
plot(t, tra(2,:) - x(3,:), 'r-', 'LineWidth', 2)
title('Position error on y')
xlabel('time (s)')
ylabel('position error (m)')
subplot(3,1,3)
plot(t, tra(3,:) - x(5,:), 'r-', 'LineWidth', 2)
title('Position error on z')
xlabel('time (s)')
ylabel('position error (m)')
grid on
saveas(gcf, 'drone_position_error_subplots.png');

figure(9);
plot(t, sqrt(x(1,:).^2 + x(3,:).^2), 'r-', 'LineWidth', 2);
hold on
plot(t, sqrt(tra(1,:).^2 + tra(2,:).^2), 'b-', 'LineWidth', 2);
title('Distance from origin');
xlabel('time (s)')
ylabel('distance (m)')
legend('drone distance', 'Desired distance');
grid on
saveas(gcf, 'drone_distance_from_origin.png');


%% video recording

% ------------ 1)  video writer  ---------------------------------------
v = VideoWriter('trajectory_following.mp4','MPEG-4');
v.FrameRate = 30;
open(v);

% ------------ 2)  figure & static background --------------------------
fig = figure(100);  clf(fig)
ax  = axes('Parent',fig);  hold(ax,'on');  grid(ax,'on');
view(ax,35,25);

% draw the entire reference once, light‑grey for context
plot3(ax,tra(1,:),tra(2,:),tra(3,:),'Color',[.7 .7 .7],'LineWidth',1);

% axes limits – change to fit your scenario
% axis(ax,[0 1.5 0 1.5 0 1.5]);   xlabel X; ylabel Y; zlabel Z
title(ax,'Crazyflie following a moving target')

% ------------ 3)  graphics objects we will update ---------------------
quad      = plot3(ax,NaN,NaN,NaN,'bo','MarkerSize',6,'MarkerFaceColor','b');
traj      = animatedline('Parent',ax,'LineWidth',1.2,'Color',[0 0.447 0.741]);

tgt       = plot3(ax,NaN,NaN,NaN,'ro','MarkerSize',6,'MarkerFaceColor','r');
tgtTrail  = animatedline('Parent',ax,'Color',[1 0 0],'LineStyle','--');

errLine   = plot3(ax,[NaN NaN],[NaN NaN],[NaN NaN],'g:','LineWidth',1);

% orientation “cross”
L = 0.08;  hArm = plot3(ax,NaN,NaN,NaN,'r-','LineWidth',2);

% ------------ 4)  animation loop --------------------------------------
Nskip = 10;                       % <- show every 10‑th solver step
for k = 1:Nskip:length(t)

    % quad position and historical track
    set(quad,'XData',x(1,k),'YData',x(3,k),'ZData',x(5,k));
    addpoints(traj,x(1,k),x(3,k),x(5,k));

    % target marker and its own trail
    set(tgt,'XData',tra(1,k),'YData',tra(2,k),'ZData',tra(3,k));
    addpoints(tgtTrail,tra(1,k),tra(2,k),tra(3,k));

    % green error vector from drone to target
    set(errLine,'XData',[x(1,k) tra(1,k)], ...
                'YData',[x(3,k) tra(2,k)], ...
                'ZData',[x(5,k) tra(3,k)]);

    % optional attitude cross (same as before)
    phi   = x(7,k);  theta = x(9,k);  psi = x(11,k);
    Rbi   = angle2dcm(psi,theta,phi);            % Aerospace TB function
    arms  = L*[ 1 0 0; -1 0 0; 0 1 0; 0 -1 0]';
    armsI = Rbi*arms;
    set(hArm,'XData',armsI(1,:)+x(1,k), ...
             'YData',armsI(2,:)+x(3,k), ...
             'ZData',armsI(3,:)+x(5,k));

    % grab frame & write
    drawnow limitrate nocallbacks
    writeVideo(v,getframe(fig));
end

close(v)