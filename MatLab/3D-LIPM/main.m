clc
close all
clear all

%% Parameters
params.timeHorizon  =  1;                                       % Seconds 
params.framerate    =  50;                                      % INTEGER
params.timestep     =  1 / params.framerate;                    % Seconds
params.Nl           =  params.timeHorizon / params.timestep;    % INTEGER
params.StepLength   =  0.15;
params.kx           =  0;
params.ky           =  0;
params.zc           =  0.5;     % m     - Approximate Height of the CoM 
params.g            =  9.81;    % ms⁻²  - Acceleration due to Gravity
params.m            =  7.4248;  % kg    - Total Mass of a NuGus
params.CP           =  1;

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
params.weights.Qe   = 1    * eye(2,2);     
params.weights.Qx   = 0    * eye(6,6);
params.weights.R    = 1e-3 * eye(2,2);

%% Model
model.t             = 0:params.timestep:10;
model.x             = zeros(6,length(model.t));
model.y             = zeros(2,length(model.t));
model.pREF          = zeros(2,length(model.t)+params.Nl);
model.u             = zeros(2,length(model.t));
params.mode         = -1;


%-% Initial Condidtions
    INDEX = 1;
    model.X0     = [0;  % x  Position
                    0;  % x' Velocity 
                    0;  % x" Acceleration
                    0;  % z  Position
                    0;  % z' Velocity 
                    0]; % z" Acceleration

    model.glbTrj = trajGen_sin(model.t,[model.X0(1); 0; model.X0(4)]);

    initFig = figure(1);
        cla(initFig)
        view(125,20);
        plotPendulum(INDEX,model,params)

        legend({'Z_0','X_0','Y_0','{\bfQ}(t) aka {\bfTraj}',...
                '{\bfCoM} (x,z)','','{\bfZMP} (x,z)','y_c = 0.5'},...
            'FontSize',14,'Location','northeast')

        title("Pendulum Initial Position",'FontSize',20);

%% STEPPING
    % Helper Functions
    gradFUNC = @(A,B) (B(2) - A(2)) ...
                     /(B(1) - A(1));  % Gradient -> ∇
    % Initialise variables
    Nl       = params.Nl;              % N# INTEGER Future Indexes
    stpLngth = params.StepLength;     % Step Size:   m
    Q        = model.glbTrj;          %         Q:  [x y z]ᵀ
    r        = 0.1;                   % Radius of Circle
    STEP     = params.mode;           % DEFINE MODE:  1 RIGHT Step 
                                      %              -1 LEFT  Step
    current_Dist    = 0;              % Accumulated Current Distance
    preview_Dist    = 0;              % Accumulated Preview Distance
    model.pREF(:,1) = model.glbTrj([1 3],1);   % Last Foot Step -> Traj Start Point
    A               = model.glbTrj(:,1);       % A = [x₁ y₁ z₁]ᵀ
    t_begin         = 2;                     % Index of Step Beginning
    for i=2:length(model.t)
        tic

        % PREVIEW FORWARD
        preview_A    = A;
        preview_STEP = STEP;
        preview_init = i + 1;
        preview_end  = i + Nl;
        preview_Dist = current_Dist;
        for p = preview_init:preview_end
            if p > length(Q)
                model.pREF(:,p) = model.pREF(:,length(Q));
            else
                preview_Dist = preview_Dist + ...
                    norm(model.glbTrj(:,p-1) - model.glbTrj(:,p));
                if preview_Dist > stpLngth
                    preview_B = model.glbTrj(:,p);
                    M = gradFUNC(preview_A([1 3]),preview_B([1 3]));
                    model.pREF(:,p) = preview_B([1 3]) + ...
                                      preview_STEP*[M*r*sqrt(1/(1+M^2)); ...
                                                     -r*sqrt(1/(1+M^2))];
                    preview_STEP  = preview_STEP * -1;
                    preview_A     = preview_B;
                    preview_Dist  = 0;
                else
                    model.pREF(:,p) = model.pREF(:,p-1);
                end
            end
        end


        current_Dist = current_Dist + ...
            norm(model.glbTrj(:,i-1) - model.glbTrj(:,i));
        
        if current_Dist > stpLngth || i == length(Q)
            % TAKE STEP
            t_end           = i;       % Index of Step Ending
            params.mode     = STEP;    % Mode
          
            for j=t_begin:t_end
               [model.y(:,j), CoMk, model] = LIPM3D(model,j,params);
            end

            % CLEAN UP
            current_Dist    = 0;
            A               = model.glbTrj(:,t_end);
            STEP            = STEP * -1;
            t_begin         = t_end;
            toc
        end
    end
%% Sanity Check
Tzmp = @(px, x, ddx) params.m * (params.g *(x - px)  - ddx*params.zc);

%% Figures
TRAJECTORIES = figure(2);
    clf(TRAJECTORIES)
    
    subplot(2,1,1)
        hold on
        grid on
        title("Pendulum ZMP X axis travel",'FontSize',16);
        subtitle({"System Freq: " + params.framerate + " Hz", ...
                  "Time Horizon: " + params.timeHorizon + " sec"}, ...
                  'Fontsize',12)
        xlabel('Time ({\it sec})','FontSize',12,'FontWeight','bold')
        ylabel('X Displacement ({\it metres})','FontSize',12,'FontWeight','bold')
        plot(model.t,model.pREF(1,1:length(model.t)),'k','LineWidth',2)
        plot(model.t,model.y(1,:),'r','LineWidth',2)

        legend({'y^{D}_{d} {\itaka} ZMP^{REF}_{X}'...
                'y_d(1) {\itaka} ZMP_{X}'...
                },...
            'FontSize',12,'Location','eastoutside')
        
    subplot(2,1,2)
        hold on
        grid on
        title("Pendulum ZMP Z axis travel",'FontSize',16);
        subtitle({"System Freq: " + params.framerate + " Hz", ...
                  "Time Horizon: " + params.timeHorizon + " sec"}, ...
                  'Fontsize',12)
        xlabel('Time ({\it sec})','FontSize',12,'FontWeight','bold')
        ylabel('Z Displacement ({\it metres})','FontSize',12,'FontWeight','bold')
        plot(model.t,model.pREF(2,1:length(model.t)),'k','LineWidth',2)
        plot(model.t,model.y(2,:),'r','LineWidth',2)
        
        legend({'y^{D}_{d}(2) {\itaka} ZMP^{REF}_{Z}'...
                'y_d(2) {\itaka} ZMP_{Z}'...
                },...
            'FontSize',12,'Location','eastoutside')

%% FRAMES

jINDEXES = [136 450];
params.CP = 0;

FRAMES = figure(4);
    cla(FRAMES)

    for i=1:2
        subplot(2,1,i)
        j = jINDEXES(i);
        view(90,90)
        hold on
        grid on
        plotPendulum(j,model,params)
        plot3(model.x(4,1:j),model.x(1,1:j),params.zc*ones(1,length(1:j)),'r:','LineWidth',2)
        plot3(model.y(2,1:j),model.y(1,1:j),zeros(1,length(1:j)),'color','#0096FF','LineWidth',2)
        plot3(model.pREF(2,:),model.pREF(1,:),zeros(1,length(model.pREF)),'k+','LineWidth',2,'MarkerSize',10)
        title("Pendulum at t = " + model.t(j) + "sec",'FontSize',16);
    legend({'Z_0','X_0','Y_0','{\bfQ}(t) aka {\bfTraj}',...
                    '{\bfCoM} (x,z)','','{\bfZMP} (x,z)','{\bfCoM} Path',...
                    '{\bfy_d} {\itaka} {\bfZMP} Path','{\bfy^{D}_d} {\itaka} {\bfZMP}^{REF}',},...
                'FontSize',14,'Location','eastoutside')
        title("Pendulum at t = " + model.t(j) + "sec",'FontSize',20);

    end
    drawnow


%% Animation
% Preallocate IMAGE

ANIMATION = figure(3);
for j=1:length(model.t)
    cla(ANIMATION) % Clear Axes
    view(125,20);
    plotPendulum(j,model,params)
    plot3(model.x(4,1:j),model.x(1,1:j),params.zc*ones(1,length(1:j)),'r:')
    plot3(model.y(2,1:j),model.y(1,1:j),zeros(1,length(1:j)),'b:','LineWidth',2)
    plot3(model.pREF(2,:),model.pREF(1,:),zeros(1,length(model.pREF)),'kx','LineWidth',2,'MarkerSize',10)
    legend({'Z_0','X_0','Y_0','{\bfQ}(t) aka {\bfTraj}',...
                '{\bfCoM} (x,z)','','{\bfZMP} (x,z)','y_c = 0.5',...
                '','{\bfy_d}','{\bfy^{D}_d}',},...
            'FontSize',14,'Location','northeast')
    title("Pendulum at t = " + model.t(j) + "sec",'FontSize',20);
    IMAGE(j) = getframe(gcf);
    drawnow
end

%% VIDEO
videoWriterObj           = VideoWriter('3D_LIPM.mp4','MPEG-4');
videoWriterObj.FrameRate = params.framerate; % 15sec video
open(videoWriterObj);                        
for i=1:length(IMAGE)
    frame = IMAGE(i);   % Convert from an Image to a Frame
    writeVideo(videoWriterObj, frame);
end
close(videoWriterObj);
