clc
clear

%% Parameters
params.timeHorizon  =  1;                                     % Seconds 
params.timestep     =  0.01;                                    % Seconds
params.Nl           =  params.timeHorizon / params.timestep;    % INTEGER
params.stepSize     =  0.125;
params.kx           =  0;
params.ky           =  0;
params.zc           =  0.495;    % m     - Approximate Height of the CoM 
params.g            = -9.81;    % ms⁻²  - Acceleration due to Gravity
params.m            =  7.4248;  % kg    - Total Mass of a NuGus

%% Weights for controller `Performance Index`
% Design of an optimal controller for a discrete-time system subject
% to previewable demand
%   1985 - KATAYAMA et al.
%       ∞     
%% Ju = Σ [ e(i)ᵀ⋅Qₑ⋅e(i) + Δx(i)ᵀ⋅Qₓ⋅Δx(i) + Δu(i)ᵀ⋅R⋅u(i) ]
%      i=k
% where:
%        e(i): ZMPₓ(i) - Yₓ     aka Tracking Error 
%       Δx(i): x(i) - x(i-1)    aka Incremental State Vector
%       Δu(i): u(i) - u(i-1)    aka Incremental Control Vector
params.weights.Qe   = 1;     % Increasing punishes ZMP Reference Error
params.weights.Qx   = [0 0 0;   % (1,1) Increasing punishes displacement
                       0 0 0;   % (2,2) Increasing punishes velocity
                       0 0 0];  % (3,3) Increasing punishes acceleration
params.weights.R    = 5e-4;     % Increasing Punish Control Action

%% Model
model.t             = 1:params.timestep:10;
model.x             = zeros(3,length(model.t));
model.y             = zeros(1,length(model.t));
model.pREF          = zeros(1,length(model.t));
model.u             = zeros(1,length(model.t));
model.gains.Gi      = zeros(1,length(model.t));
model.gains.Gx      = zeros(1,length(model.t));
model.gains.Gd      = zeros(1,length(model.t));

%% Initial Condidtions
    model.X0     = [0;                      % Position
                    0;                      % Velocity 
                    0];                     % Acceleration
    model.pREF = pREF(model.t, params);     % Load ZMPₓ Reference

%% Simulation Loop
for i=1:length(model.t)
    [ZMPk, CoMk, model] = LIPM2D(model,i,params);
end

%% Sanity Checker
Tzmp = @(px, x, ddx) params.m * (params.g *(x - px)  - ddx*params.zc);

%% Figure
TRAJECTORIES = figure(1);
    clf
    hold on
    grid on
    plot(model.t,Tzmp(model.y,model.x(1,:),model.x(3,:)),...
        'm-','LineWidth',2)
    plot(model.t,model.pREF,"k--","LineWidth",2)
    plot(model.t,model.y,"g-","LineWidth",2)
    plot(model.t,model.x(1,:), "r-","LineWidth",2)
    legend({'{\tau_{x}} about ZMP_{x}','ZMP Ref','ZMP_{x}','CoM_x'},...
        "Location","east",'FontSize',11)
    ylabel("{\bfDisplacement X} ({\itmetres})");
    xlabel("{\bfTime} ({\itsecs}) \newline{\bfPeriod = "            + ...
            num2str(params.timestep)                                + ...
            "} ({\itsecs})");
    title("Discretised Linear Inverted Pendulum \newline... with "  + ...
           num2str(params.timeHorizon)                              + ...
           " second Time Horizon");

%% Animation
ANIMATION = figure(2);
    hold on
    grid on
    view(30,30)
    axis([-0.2 0.8 -0.5 0.5 0 0.7]);
    for j=1:length(model.t)
    cla(ANIMATION) % Clear Axes
    % X Y Z
        plot3([0 1], [0 0], [0 0], "r", "LineWidth",2); % X Vector
        plot3([0 0], [0 1], [0 0], "g", "LineWidth",2); % Y Vector
        plot3([0 0], [0 0], [0 1], "b", "LineWidth",2); % Z Vector
    % Constraint Plane - CoM Bound to this plane
        Z     = @(X,Y) params.kx*X + params.ky*Y + params.zc; 
        [X,Y] = meshgrid(-1:0.1:1);
        plot3([0 0], [-0.1 -0.1], [0 params.zc], ...
              "r--", "LineWidth",2,"MarkerSize",10); % Zc Dist
        surf(X,Y,Z(X,Y), ...
             "FaceColor","r","LineStyle","none","FaceAlpha",0.1);
    % Animation -> Pendulum
        plot3(              model.y(:,j),      0,            0,"kx",...
             "LineWidth",2,"MarkerSize",5);
        plot3(              model.x(1,j),      0,    params.zc,"ko",...
             "LineWidth",2,"MarkerSize",10);
        plot3(               model.x(1,j),     0,    params.zc,"rx",...
             "LineWidth",2,"MarkerSize",2);
        plot3([model.y(:,j) model.x(1,j)], [0 0],[0 params.zc],"k-",...
             "LineWidth",2)
    % Legend
        title("Discretised LIPM Animation: " + model.t(j) + " sec")
        legend("X+","Y+","Z+",               ... Direction Vectors
               "Z_{c} Height","Z_{c} Plane", ... Zc Plane
               "ZMP_{x}","CoM_{x}",          ... ZMP + CoM
               "Location","east")
    end