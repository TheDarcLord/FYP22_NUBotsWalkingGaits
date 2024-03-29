clear all
close all
clc
%% NuGus Quasi-Static Walk - Local Coordinates
 % Based on Igus robot lower half ...
    params.enableComms = 1;
    IP_ADDRESS = "10.0.0.127"; % OR 10.1.1.3

%% Video & Time Parameters
    model.frequency   = 40;                      % FPS
    params.timestp    = model.frequency^(-1);    % Seconds
    model.tspan       = 0 : params.timestp : 120; % [ time ]
 % Step Parameterisation - Trajectory dictates time                      |
    params.vMax       = 0.12;   % m/s  - Norm vCoM Max                   |
    params.zc         = 0.42;   % m    - Height of the CoM ^             |
    params.m          = 7.4248; % kg   - Total Mass of a NuGus           |
    params.StepLength = 0.15;   % m    - 15 cm Step Forward              |
    params.StepHeight = 0.07;   % m    - 8 cm Step Height                |
 % ______________________________________________________________________|
 % Joint Bounds
    params.lb      = [-pi/2; -pi/2;  1e-4; -pi/2; -pi/2; -pi/2; ...
                      -pi/2; -pi/2; -pi/2; -pi/2; -pi/2; -pi/2];
    params.ub      = [ pi/2;  pi/2;  pi/2;  pi/3;  pi/2; +pi/2; ...
                      +pi/2;  pi/2;  pi/2;  1e-4;  pi/2;  pi/2];
 % Displacements
    params.fibula     = 0.19942;  % m    - Lower leg NUgus
    params.femur      = 0.19954;  % m    - Upper Leg NUgus
    params.HipWidth   = 0.13;     % m    - Pelvis
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
    params.mass.foot   = 0.2023;                    % Foot

%% Model setup
 % Step Mode -
    model.mode   = zeros(1,length(model.tspan));
    params.mode  = -1;       % LEFT  FIXED - FKM T16
    %               0;       % BOTH  FIXED - FKM T1H T6H
    %               1;       % RIGHT FIXED - FKM T61
 % Robot -
    model.q      = zeros(12,length(model.tspan)); % q        [θ₁θ₂θ₃ ...]ᵀ
    model.xe     = zeros(6,length(model.tspan));  % xe           [XYZϕθΨ]ᵀ
    model.r0CoMg = zeros(3,length(model.tspan));  % r0CoMg          [XYZ]ᵀ
    model.pREF   = zeros(2,length(model.tspan));  % pREF [REFx REFy REFz]ᵀ      

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
    model.xe(:,1)     = k(model.q0, params);
    model.r0CoMg(:,1) = rCoM(model.q0,params);
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
        for i=1:38
            commJointValues(IP_ADDRESS,"10013",[CNV.*model.q0; (1/(39 - i)) * iGusArms]);
        end
        commJointValues(IP_ADDRESS,"10013",[CNV.*model.q0; iGusArms]);
    end

%% Generate Trajectory
    model.glbTrj = trajGenC_dribble(params.timestp, model.xe(1:3,1)./2);
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
    model.pREF(:,1) = Q([1 3],1);        % Last Foot Step -> TrajStartPoint
    A               = model.glbTrj(:,1); % A = [x₁ y₁ z₁]ᵀ
    t_init          = 1;                 % Initialise Index
    model.TBE       = eye(4);            % HomoTrans: Base -> End Effector
    for i=2:length(model.tspan)
        tic
        accuDist = accuDist + norm(Q(:,i-1) - Q(:,i));

        if accuDist > params.StepLength || i == length(Q)
            % STEP TIME INDEXES
            t_run              = t_init + 1;   % `j` runs the step
            t_end              = i;            % Index of Step Ending
            params.mode        = STEP;         % Mode
            model.xe(:,t_init) = k(model.q(:,t_init), params); % Update Xe
            
            % DESIGN STEP
            B = model.glbTrj(:,i);
            M = gradFUNC(A([1 3]),B([1 3]));
            % Right (+) & Left (-)
            model.pREF(:,i) = B([1 3]) + STEP*[M*r*sqrt(1/(1+M^2)); ...
                                                -r*sqrt(1/(1+M^2))];

            % GENERATE STEP TRAJECTORY
            Qstep(:,t_run:t_end) = trajGenStep(model.xe(:,t_init), ...
                                               model.pREF(:,t_end), ...
                                               t_run:t_end, ...
                                               model,params);

            DEBUG(t_init,t_init,t_end,Qstep,model,params);

            for j=t_run:t_end
                jNEG            = j - 1;
                xeSTAR          = Qstep(:,j);
                model.mode(:,j) = params.mode;
                model.q(:,j)    = k_Inv(model.q(:,jNEG), ... q0
                                                 xeSTAR, ... xe*
                                                 j, model, params);
               [model.xe(:,j), model.TBE] = k(model.q(:,j), params);
                model.r0CoMg              = rCoM(model.q(:,j),params);
                
                DEBUG(j,t_run,t_end,Qstep,model,params);
                if params.enableComms == 1
                    commJointValues(IP_ADDRESS,"10013",[CNV.*model.q(:,j); iGusArms]);
                end
            end

            for u=t_init:length(model.tspan)
                % UPDATE: Global Trajectory to End Effector Coords
                model.glbTrj(:,u) = ...
                    updateCoord(model.TBE, model.glbTrj(:,u));
                tmpREF = [model.pREF(1,u); 0; model.pREF(2,u)];
                tmpREF = ...
                    updateCoord(model.TBE, tmpREF);
                model.pREF(:,u) = tmpREF([1 3]);
            end

            % CLEAN UP
            STEP      = STEP * -1;
            accuDist  = 0;
            A         = model.glbTrj(:,i);
            t_init    = t_end;
            toc
        end
    end


%% Animation
    ROBOT_FRAME = figure(1);
    
    hold on
    grid on
    axis equal
    view(-165,45);
    set(gca,'Color','#CCCCCC');
    title("3D Model - ZMP Walking",'FontSize',12);
    xlabel('{\bfZ} (metres)');
    ylabel('{\bfX} (metres)');
    zlabel('{\bfY} (metres)');

    for i=1:length(model.tspan)
        cla(ROBOT_FRAME)
        params.mode = model.mode(i);
        [~] = plotRobot(i,model,params);
        [~] = plotSteps(model);
        
        IMAGE(i) = getframe(gcf);
    end

%% Send Joint [ Values; Arms ]
if params.enableComms == 1
    for i=1:38
        commJointValues(IP_ADDRESS,"10013",...
            [CNV.*model.q0; (1/(39 - i)) * iGusArms]);
    end
    commJointValues(IP_ADDRESS,"10013",[CNV.*model.q0; iGusArms]);

    for i=1:3340
        commJointValues(IP_ADDRESS,"10013",[CNV.*model.q(:,i); iGusArms]);
        pause(0.1);
    end
end
%% VIDEO
videoWriterObj           = VideoWriter('3D_Step.mp4','MPEG-4');
videoWriterObj.FrameRate = model.frequency;
open(videoWriterObj);                        
for i=1:length(IMAGE)
    frame = IMAGE(i);
    writeVideo(videoWriterObj, frame);
end
close(videoWriterObj);
