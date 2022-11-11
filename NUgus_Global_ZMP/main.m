clear all
close all
clc 
%% NuGus ZMP Walk - Local Coordinates
 % Based on Igus robot lower half ...
    params.enableComms = 0;

%% Video & Time Parameters
    params.frequency  = 200;                             % FPS
    params.timestp    = params.frequency^(-1);           % Seconds
    model.tspan       = 0 : params.timestp : 30;         % [ time ]
    model.timeHrzn    = 1;                               % Seconds
    model.Nl          = model.timeHrzn / params.timestp; % INTEGER
 % Weights for controller `Performance Index`
    % Design of an optimal controller for a discrete-time system subject
    % to previewable demand
    %   1985 - KATAYAMA et al.
    %      ∞     
    % Jᵤ = Σ [ e(i)ᵀ⋅Qₑ⋅e(i) + Δx(i)ᵀ⋅Qₓ⋅Δx(i) + Δu(i)ᵀ⋅R⋅u(i) ]
    %     i=k
    % where:
    %        e(i): ZMPₓ(i) - Yₓ     aka Tracking Error              Qe
    %       Δx(i): x(i) - x(i-1)    aka Incremental State Vector    Qx
    %       Δu(i): u(i) - u(i-1)    aka Incremental Control Vector  R
    params.weights.Qe   = 1.00 * eye(2,2);     
    params.weights.Qx   = 0    * eye(6,6);
    params.weights.R    = 1e-4 * eye(2,2);
 % Physical Parameters - Affect CoM or FKM
    params.kx           = 0;        % These affect the plane to which    |
    params.ky           = 0;        % ... the CoM is constrained         |
    params.zc           = 0.42;     % m    - Height of the CoM ^         |
    params.g            = 9.81;     % ms⁻² - Acceleration due to Gravity |
    params.m            = 7.4248;   % kg   - Total Mass of a NuGus       |
    params.StepLength   = 0.05;     % m    - 15 cm Step Forward          |
    params.StepHeight   = 0.05;     % m    - 10 cm Step Height           |
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
    model.r.q      = zeros(12,length(model.tspan)); % q   [θ₁θ₂θ₃ ...]ᵀ
    model.r.xe     = zeros(6,length(model.tspan));  % xe      [XYZϕθΨ]ᵀ
    model.r.r0Lg   = model.r.xe;                    % r0EL    [XYZϕθΨ]ᵀ
    model.r.r0Rg   = model.r.xe;                    % r0ER    [XYZϕθΨ]ᵀ
    model.r.r0Hg   = model.r.xe;                    % r0H     [XYZϕθΨ]ᵀ
    model.r.r0CoMg = zeros(3,length(model.tspan));  % r0CoMg  [XYZ]ᵀ
 % Pendulum
    model.p.x      = zeros(6,length(model.tspan));  % Xcom      [x x' x"]ᵀ
    model.p.y      = zeros(2,length(model.tspan));  % pₓ₂     [ZMPx ZMPz]ᵀ
    model.p.pREF   = model.p.y;                     % pₓ₂REF  [REFx REFz]ᵀ
    model.p.u      = model.p.y;                     % Uₓ₂         [Ux Uz]        

%% Initial Position & Orientation
    model.r.q0 = deg2rad([ -05.5;   % θ₁
                           -39.5;   % θ₂  ~ 2D θ₁ Ankle
                            56.5;   % θ₃  ~ 2D θ₂ Knee 
                           -17.0;   % θ₄  ~ 2D θ₃ Hip
                            05.5;   % θ₅
                            00.0;   % θ₆
                            00.0;   % θ₇
                            05.5;   % θ₈
                            17.0;   % θ₉  ~ 2D θ₄ Hip
                           -56.5;   % θ₁₀ ~ 2D θ₅ Knee
                            39.5;   % θ₁₁ ~ 2D θ₆ Ankle
                           -05.5]); % θ₁₂
    CNV = [-1;+1;+1;+1;-1;+1;-1;+1;-1;-1;-1;+1]; % Joint Angle Conversion

    model.r.q(:,1)      = model.r.q0;
   [model.r.xe(:,1),   model.r.r0Lg(:,1), ...% F
    model.r.r0Rg(:,1), model.r.r0Hg(:,1)] ...% K
        = k(model.r.q0, 1, model, params);   % M

    model.r.r0CoMg(:,1) = rCoM(model.r.q0,1,model,params);
    model.p.x(:,1) = [model.r.r0CoMg(1,1); 0; 0;  % Position X
                      model.r.r0CoMg(3,1); 0; 0]; % Position Z
    model.mode(1)       = params.mode;

    iGusArms            = deg2rad([105; -15; -100; -100; 15; 105]);

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
        commJointValues("10.1.1.3","10013",[CNV.*model.r.q0; iGusArms]);
    end


%% Generate Trajectory
    model.trajInit = (model.r.r0Rg(1:3,1) + model.r.r0Lg(1:3,1))./2;
    model.glbTrj = ...
        trajGen_sin(model.tspan, model.trajInit);
%        trajGenC(1/params.framerate, ...       % Time Span
%                 model.r.xe(1:3,1)./2);          % Init Position
    [~] = plotSteps(model);

%% STEPPING
    % Helper Functions
    gradFUNC = @(A,B) (B(2) - A(2)) ...
                     /(B(1) - A(1));  % Gradient -> ∇
    % Initialise variables
    Nl       = model.Nl;              % N# INTEGER Future Indexes
    stpLngth = params.StepLength;     % Step Size:   m
    Q        = model.glbTrj;          %         Q:  [x y z]ᵀ
    Qstep    = zeros(6,length(Q));    % Qstep:  [x 0 z]ᵀ
    r        = (params.HipWidth/2) + 0.02;   % Radius of Circle
    STEP     = params.mode;           % DEFINE MODE:  1 RIGHT Step 
                                      %              -1 LEFT  Step
    current_Dist = 0;                 % Accumulated Current Distance
    preview_Dist = 0;                 % Accumulated Preview Distance
    if STEP == 1
        model.p.pREF(:,1) = model.r.r0Lg([1 3],1);   % Last Foot Step
    else
        model.p.pREF(:,1) = model.r.r0Rg([1 3],1);   % Last Foot Step
    end
    A        = model.glbTrj(:,1);     % A = [x₁ y₁ z₁]ᵀ
    t_init   = 1;                     % Index of Step Beginning
    for i=2:length(model.tspan)
        tic

        current_Dist = current_Dist + ...
            norm(model.glbTrj(:,i-1) - model.glbTrj(:,i));

        % PREVIEW FORWARD
        preview_A    = A;
        preview_STEP = STEP;
        preview_init = i + 1;
        preview_end  = i + Nl;
        preview_Dist = current_Dist;
        for p = preview_init:preview_end
            if p > length(Q)
                model.p.pREF(:,p) = model.p.pREF(:,length(Q));
            else
                preview_Dist = preview_Dist + ...
                    norm(model.glbTrj(:,p-1) - model.glbTrj(:,p));

                if preview_Dist > stpLngth 
                    preview_B = model.glbTrj(:,p);
                    M = gradFUNC(preview_A([1 3]),preview_B([1 3]));
                    model.p.pREF(:,p) = preview_B([1 3]) + ...
                                        preview_STEP*[M*r*sqrt(1/(1+M^2)); ...
                                                       -r*sqrt(1/(1+M^2))];
                    preview_STEP  = preview_STEP * -1;
                    preview_A     = preview_B;
                    preview_Dist  = 0;
                else
                    model.p.pREF(:,p) = model.p.pREF(:,p-1);
                end
            end
        end
        
        if current_Dist > stpLngth || i == length(Q)
            % STEP TIME INDEXES
            t_run       = t_init + 1;               % `j` runs the step
            t_end       = i;                        % Index of Step Ending
            params.mode = STEP;                     % Change Step Mode
            model = LIPM3D(model, t_init, params);  % ZMP @ time(init)
            model.r.xe(:,t_init) = ...
                k(model.r.q(:,t_init), t_init, model, params);
            % GENERATE STEP TRAJECTORY
            Qstep(:,t_run:t_end) = trajGenStep(model.r.xe(:,t_init),  ...
                                               model.p.pREF(:,t_end), ...
                                               t_run:t_end, ...
                                               model,params);
            
            DEBUG(t_init,t_run,t_end,Qstep,model,params)
            % SOLVE IKM and run time
            for j=t_run:t_end
                jNEG           = j - 1;             % t_init ... t_run ...
                xeSTAR         = Qstep(:,j);        % End Effector
                model.mode(j)  = params.mode;       % Record Mode
                model = LIPM3D(model, j, params);   % ZMP @ time(j)
                model.r.q(:,j) = k_Inv(model.r.q(:,jNEG), ... q0
                                                  xeSTAR, ... xe*
                                                  j, model, params);
               [model.r.xe(:,j),   model.r.r0Lg(:,j),  ...  % F
                model.r.r0Rg(:,j), model.r.r0Hg(:,j)]  ...  % K
                  = k(model.r.q(:,j), jNEG, model, params); % M
                model.r.r0CoMg(:,j) = rCoM(model.r.q(:,j), j, model, params);
                
                DEBUG(j,t_run,t_end,Qstep,model,params);
            end

            % CLEAN UP
            current_Dist    = 0;
            STEP            = STEP * -1;
            t_init          = t_end;
            A               = model.glbTrj(:,t_end);
        end
        toc
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
    view(145,30);
    [~] = plotRobot(i,model,params);

    a = 0.5;
    for i=1:length(model.tspan)
        cla(ROBOT_FRAME)
        CM = model.r.r0CoMg([1 3],i);
        axis([ CM(2)-a, CM(2)+a, CM(1)-a, CM(1)+a, 0.0, 1.0]);
        [~] = plotRobot(i,model,params);
        [~] = plotSteps(model);
        [~] = plotPend(i,model,params);
        
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
