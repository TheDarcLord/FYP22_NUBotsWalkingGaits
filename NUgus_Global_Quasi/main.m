clear all
close all
clc 
%% NuGus ZMP Walk - Local Coordinates
 % Based on Igus robot lower half ...
    params.enableComms = 0;

%% Video & Time Parameters
    params.frequency  = 40;                              % FPS
    params.timestp    = params.frequency^(-1);           % Seconds
    model.tspan       = 0 : params.timestp : 30;         % [ time ]
    model.timeHrzn    = 1;                               % Seconds
 % Physical Parameters - Affect CoM or FKM
    params.vMax         = 0.1;      %
    params.zc           = 0.42;     % m    - Height of the CoM ^         |
    params.m            = 7.4248;   % kg   - Total Mass of a NuGus       |
    params.StepLength   = 0.10;     % m    - 15 cm Step Forward          |
    params.StepHeight   = 0.07;     % m    - 10 cm Step Height           |
% _______________________________________________________________________|
% Joint Bounds
    params.lb      = [-pi/2; -pi/2;  1e-4; -pi/2; -pi/2; -pi/2; ...
                      -pi/2; -pi/2; -pi/2; -pi/2; -pi/2; -pi/2];
    params.ub      = [ pi/2;  pi/2;  pi/2;  pi/3;  pi/2; +pi/2; ...
                      +pi/2;  pi/2;  pi/2;  1e-4;  pi/2;  pi/2];
% Displacements
    params.fibula     = 0.19942;  % m    - Lower leg NUgus
    params.femur      = 0.19954;  % m    - Upper Leg NUgus
    params.HipWidth   = 0.11;     % m    - Pelvis
    params.heel2ankle = 0.038;    % m    - Tarsal
    params.ankle2knee = [0.005669; 0.19942; -0.0002];
    params.knee2hip   = [       0; 0.19954; -0.0015];
    params.hip2waist  = [-0.06895; 0.04625; -0.0545];
    params.sole2SP    = [ 0.022; 0; 0.011]; % m - Sole to Support Polygon
    params.ServoSize  = 0.05;     % m    - Approximation/Spacing
 % Masses
    params.mass.fibula = 0.1719;                    % Paired with `tibia`
    params.mass.femur  = 0.3423;                    % Thigh Bone
    params.mass.joint  = 0.300;                     % Knee Bone / Joints
    params.mass.pelvis = 2.9549 + 697.6 + 753.5;    % Waist
    params.mass.foot   = 0.2023;     

%% Model setup
 % Step Mode -
    model.mode   = zeros(1,length(model.tspan));
    params.mode  = -1;       % LEFT  FIXED - FKM T16
    %               0;       % BOTH  FIXED - FKM T1H T6H
    %               1;       % RIGHT FIXED - FKM T61
 % Robot
    model.q      = zeros(12,length(model.tspan)); % q   [θ₁θ₂θ₃ ...]ᵀ
    model.xe     = zeros(6,length(model.tspan));  % xe      [XYZϕθΨ]ᵀ
    model.r0Lg   = model.xe;                    % r0EL    [XYZϕθΨ]ᵀ
    model.r0Rg   = model.xe;                    % r0ER    [XYZϕθΨ]ᵀ
    model.r0Hg   = model.xe;                    % r0H     [XYZϕθΨ]ᵀ
    model.r0CoMg = zeros(3,length(model.tspan));  % r0CoMg  [XYZ]ᵀ
    model.pREF   = zeros(2,length(model.tspan));

%% Initial Position & Orientation
%     model.q0 = deg2rad([ -05.5;   % θ₁
%                          -39.5;   % θ₂  ~ 2D θ₁ Ankle
%                           56.5;   % θ₃  ~ 2D θ₂ Knee 
%                          -17.0;   % θ₄  ~ 2D θ₃ Hip
%                           05.5;   % θ₅
%                           00.0;   % θ₆
%                           00.0;   % θ₇
%                           05.5;   % θ₈
%                           17.0;   % θ₉  ~ 2D θ₄ Hip
%                          -56.5;   % θ₁₀ ~ 2D θ₅ Knee
%                           39.5;   % θ₁₁ ~ 2D θ₆ Ankle
%                          -05.5]); % θ₁₂
    model.q0 = deg2rad([ -05.5;   % θ₁
                         -15.0;   % θ₂  ~ 2D θ₁ Ankle
                          30.0;   % θ₃  ~ 2D θ₂ Knee 
                         -15.0;   % θ₄  ~ 2D θ₃ Hip
                          05.5;   % θ₅
                          00.0;   % θ₆
                          00.0;   % θ₇
                          05.5;   % θ₈
                          15.0;   % θ₉  ~ 2D θ₄ Hip
                         -30.0;   % θ₁₀ ~ 2D θ₅ Knee
                          15.0;   % θ₁₁ ~ 2D θ₆ Ankle
                         -05.5]); % θ₁₂
    CNV = [-1;+1;+1;+1;-1;+1;-1;+1;-1;-1;-1;+1]; % Joint Angle Conversion

    model.q(:,1)      = model.q0;
   [model.xe(:,1),   model.r0Lg(:,1), ...% F
    model.r0Rg(:,1), model.r0Hg(:,1)] ...% K
        = k(model.q0, 1, model, params);   % M

    model.r0CoMg(:,1) = rCoM(model.q0,1,model,params);
    model.mode(1)     = params.mode;

    iGusArms          = deg2rad([105; -15; -100; -100; 15; 105]);

    % +-+-+-+-+-+-+-+-+-+-+-+
    ROBOT_FRAME = figure(1);
        cla(ROBOT_FRAME)
        hold on
        grid on
        axis equal
        view(-210,30);
        set(gca,'Color','#CCCCCC');
        title("3D Model - ZMP Walking",'FontSize',12);
        xlabel('{\bfZ} (metres)');
        ylabel('{\bfX} (metres)');
        zlabel('{\bfY} (metres)');
        [~] = plotRobot(1,model,params);
    % +-+-+-+-+-+-+-+-+-+-+-+

    if params.enableComms == 1
        commJointValues("10.1.1.3","10013",[CNV.*model.q0; iGusArms]);
    end


%% Generate Trajectory
    model.trajInit = (model.r0Rg(1:3,1) + model.r0Lg(1:3,1))./2;
    model.glbTrj = ...
        trajGen_sin(model.tspan, model.trajInit);
%        trajGenC(1/params.framerate, ...       % Time Span
%                 model.xe(1:3,1)./2);          % Init Position
    [~] = plotSteps(model);

%% STEPPING
    % Helper Functions
    gradFUNC = @(A,B) (B(2) - A(2)) ...
                     /(B(1) - A(1));  % Gradient -> ∇
    % Initialise variables
    Q        = model.glbTrj;                   % Q:         [x y z]ᵀ
    Qstep    = zeros(6,length(model.glbTrj));  % Qstep:     [xe]
    r        = (params.HipWidth/2) + 0.03;     % Radius of Circle + shim
    STEP     = params.mode;              % DEFINE MODE:  1 RIGHT Step 
                                         %              -1 LEFT  Step
    accuDist        = 0;                 % Accumulated Distance
    A               = model.glbTrj(:,1); % A = [x₁ y₁ z₁]ᵀ
    t_init          = 1;                 % Initialise Index
    if STEP == 1
        model.pREF(:,1) = model.r0Lg([1 3],1);   % Last Foot Step
    else
        model.pREF(:,1) = model.r0Rg([1 3],1);   % Last Foot Step
    end
    
    for i=2:length(model.tspan)
        tic
        accuDist = accuDist + norm(Q(:,i-1) - Q(:,i));

        if accuDist > params.StepLength|| i == length(Q)
            % STEP TIME INDEXES
            t_run       = t_init + 1;               % `j` runs the step
            t_end       = i;                        % Index of Step Ending
            params.mode = STEP;                     % Change Step Mode
            model.xe(:,t_init) = ...
                k(model.q(:,t_init), t_init, model, params);

            % DESIGN STEP
            B = model.glbTrj(:,i);
            M = gradFUNC(A([1 3]),B([1 3]));
            % Right (+) & Left (-)
            model.pREF(:,i) = B([1 3]) + STEP*[M*r*sqrt(1/(1+M^2)); ...
                                                -r*sqrt(1/(1+M^2))];

            % GENERATE STEP TRAJECTORY
            Qstep(:,t_run:t_end) = trajGenStep(model.xe(:,t_init),  ...
                                               model.pREF(:,t_end), ...
                                               t_run:t_end, ...
                                               model,params);
            
            DEBUG(t_init,t_run,t_end,Qstep,model,params)

            % SOLVE IKM and run time
            for j=t_run:t_end
                jNEG           = j - 1;             % t_init ... t_run ...
                xeSTAR         = Qstep(:,j);        % End Effector
                model.mode(j)  = params.mode;       % Record Mode
                model.q(:,j)   = k_Inv(model.q(:,jNEG), ... q0
                                                xeSTAR, ... xe*
                                                j, model, params);
               [model.xe(:,j),   model.r0Lg(:,j),  ...      % F
                model.r0Rg(:,j), model.r0Hg(:,j)]  ...      % K
                  = k(model.q(:,j), jNEG, model, params);   % M
                model.r0CoMg(:,j) = rCoM(model.q(:,j), j, model, params);
                
                DEBUG(j,t_run,t_end,Qstep,model,params);
            end

            % CLEAN UP
            accuDist = 0;
            t_init   = t_end;
            STEP     = STEP * -1;
            A        = model.glbTrj(:,t_end);
        else
            model.pREF(:,i) = model.pREF(i-1);
        end
    end

%% Animation
    ROBOT_FRAME = figure(1);
    
    hold on
    grid on
    set(gca,'Color','#CCCCCC');
    title("3D Model - ZMP Walking",'FontSize',12);
    xlabel('{\bfZ} (metres)');
    ylabel('{\bfX} (metres)');
    zlabel('{\bfY} (metres)');
    axis equal
    view(90,90);
    [~] = plotRobot(i,model,params);

    a = 0.5;
    for i=1:length(model.tspan)
        cla(ROBOT_FRAME)
        CM = model.r0CoMg([1 3],i);
        axis([ CM(2)-a, CM(2)+a, CM(1)-a, CM(1)+a, 0.0, 1.0]);
        [~] = plotRobot(i,model,params);
        [~] = plotSteps(model);
        
        IMAGE(i) = getframe(gcf);
    end

%% VIDEO
videoWriterObj           = VideoWriter('3D_Step.mp4','MPEG-4');
videoWriterObj.FrameRate = params.frequency; % 15sec video
open(videoWriterObj);                        
for i=1:length(IMAGE)
    frame = IMAGE(i);   % Convert from an Image to a Frame
    writeVideo(videoWriterObj, frame);
end
close(videoWriterObj);
