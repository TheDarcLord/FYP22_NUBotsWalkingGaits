clc
clear

%% Parameters
params.timestep     = 0.01; % Seconds
params.Nl           = 5;
params.kx           = 0;
params.ky           = 0;
params.zc           = 0.814;
params.g            = -9.81;
params.m            = 1;
% Weights   
params.weights.Qe   = 1e-0;             % Punish ZMP Ref
params.weights.Qx   = 1e-1*eye(3,3);    % Punish Increment in State
params.weights.R    = 1e-4;             % Punish Control Action

%% Model
model.t             = 1:params.timestep:10;
model.x             = zeros(3,length(model.t));
model.y             = zeros(1,length(model.t));
model.pREF          = zeros(1,length(model.t));
model.u             = zeros(1,length(model.t));

%% Initial Condidtions
    model.X0     = [0;  % Position
                    0;  % Velocity 
                    0]; % Acceleration

    for i=1:length(model.t)
        model.pREF(i) = pREF(model.t(i));
    end

%% Loop
for k=1:length(model.t)-1
    model.u(:,k)   =   uk(model,k,params);
    model.y(:,k)   =    y(model.x(:,k),               params);
    model.x(:,k+1) = xKp1(model.x(:,k), model.u(:,k), params);
end

%% Figure
figure(1)
    clf
    hold on
    grid on
    plot(model.t,model.pREF,'k--','LineWidth',2)
    plot(model.t,model.y,'g-','LineWidth',2)
    plot(model.t,model.x(1,:), 'r-','LineWidth',2)
    legend('ZMP Ref','ZMP','CoM')


Z = @(X,Y) params.kx*X + params.ky*Y + params.zc; % CoM Bound to this plane
[X,Y] = meshgrid(-2.5:0.1:2.5);
%% Animation
figure(2)
    hold on
    grid on
    view(45,45)
    axis([-2.5 2.5 -2.5 2.5 0 2]);
    for j=1:length(model.t)-1
        title(model.t(j) + " sec")
        cla % Clear Axes
        % X Y Z
        plot3([0 1], [0 0], [0 0], 'r', 'LineWidth',1.5); % X
        plot3([0 0], [0 1], [0 0], 'g', 'LineWidth',1.5); % Y
        plot3([0 0], [0 0], [0 1], 'b', 'LineWidth',1.5); % Z
        % Constraint Plane
        plot3(0, 0, params.zc, 'rx', 'LineWidth',2,'MarkerSize',10); % Z
        surf(X,Y,Z(X,Y),'FaceColor','r','LineStyle','none','FaceAlpha',0.1);
        % Animation -> Pendulum
        plot3(model.y(:,j),0,0,'kx','LineWidth',2,'MarkerSize',5);
        plot3(model.x(1,j),0,params.zc,'ko','LineWidth',2,'MarkerSize',10);
        plot3([model.y(:,j) model.x(1,j)],[0 0],[0 params.zc],'k-','LineWidth',1)
        pause(0.001);
    end