clc
clear

%% Parameters
kx  = 0;
ky  = 0;
zc  = 1;
g   = 9.81;
m   = 1;

%% Constraint Plane
Z = @(X,Y) kx*X + ky*Y + zc;    % CoM Bound to this plane

%% Dynamics
py = @(y,yddot) y - (zc/g)*yddot;
px = @(x,xddot) x - (zc/g)*xddot;

%% Figure
figure('Name','LIPM')
    hold on
    grid on
    view(45,45)
    axis([-2 2 -2 2 0 2]);
    plot3([0 1], [0 0], [0 0], 'r', 'LineWidth',2); % X
    plot3([0 0], [0 1], [0 0], 'g', 'LineWidth',2); % Y
    plot3([0 0], [0 0], [0 1], 'b', 'LineWidth',2); % Z

    % Plane
    [X,Y] = meshgrid(-2:0.1:2);
    plot3(0, 0, zc, 'rx', 'LineWidth',2); % Z
    surf(X,Y,Z(X,Y),'FaceColor','r','LineStyle','none','FaceAlpha',0.3);