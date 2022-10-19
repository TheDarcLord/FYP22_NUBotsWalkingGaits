clear all
close all
clc 

%% Video & Time Parameters
    params.framerate  = 25;                             % FPS
    model.timestp     = params.framerate^(-1);          % Seconds
    model.tspan       = 0 : model.timestp : 200;        % [ time ]
 % Physical Parameters - Affect CoM or FKM
    params.zc        = 0.5;     % m    - Height of the CoM ^         |
    params.g         = 9.81;     % ms⁻² - Acceleration due to Gravity |
    params.m         = 7.4248;   % kg   - Total Mass of a NuGus       |
 % -------------------------------------------------------------------|
    params.fibula       = 0.4;      % m    - Lower leg
    params.femur        = 0.4;      % m    - Upper Leg
    params.HipWidth     = 0.2;      % m    - Pelvis
    params.ServoSize    = 0.05;     % m    - Approximation/Spacing
    params.StepLength   = 0.15;     % m    - 15 cm Step Forwards
    params.StepHeight   = 0.08;     % m    - 8 cm Step Upwards
 % Masses
    params.mass.fibula = 1.5;    % Paired with `tibia`
    params.mass.femur  = 1.5;    % Thigh Bone
    params.mass.joint  = 0.5;    % Knee Bone / Joints
    params.mass.pelvis = 1.5;    % Waist

%% Model setup
 % Steps
    % Stepping mode... Array!? ... ?
    model.mode     = zeros(1,length(model.tspan));
    params.mode    = -1;       % LEFT  FIXED - FKM T16
    %                 0;       % BOTH  FIXED - FKM T1H T6H
    %                 1;       % RIGHT FIXED - FKM T61
 % Robot
    model.q      = zeros(12,length(model.tspan)); % q   [θ₁θ₂θ₃ ...]ᵀ
    model.xe     = zeros(6,length(model.tspan));  % xe      [XYZϕθΨ]ᵀ
    model.r0Lg   = model.xe;                    % r0EL    [XYZϕθΨ]ᵀ
    model.r0Rg   = model.xe;                    % r0ER    [XYZϕθΨ]ᵀ
    model.r0Hg   = model.xe;                    % r0H     [XYZϕθΨ]ᵀ
    model.r0CoMg = zeros(3,length(model.tspan));  % r0CoMg  [XYZ]ᵀ
    model.pREF   = zeros(2,length(model.tspan));  % pₓ₂     [ZMPx ZMPz]ᵀ       

%% Initial Position & Orientation
    model.r0Lg(:,1) = [0; 0;  params.HipWidth/2;0;0;0];
    model.r0Rg(:,1) = [0; 0; -params.HipWidth/2;0;0;0];
    model.q0        = [0;    % θ₁    
                    -pi/12;    % θ₂    ->  2D θ₁ Ankle
                   2*pi/12;    % θ₃    ->  2D θ₂ Knee
                    -pi/12;    % θ₄    ->  2D θ₃ Hip
                         0;    % θ₅
                         0;    % θ₆
                         0;    % θ₇
                         0;    % θ₈
                     pi/12;    % θ₉    ->  2D θ₄ Hip
                  -2*pi/12;    % θ₁₀   ->  2D θ₅ Knee
                     pi/12;    % θ₁₁   ->  2D θ₆ Ankle
                         0];   % θ₁₂
    model.q(:,1)        = model.q0;
   [model.xe(:,1),   model.r0Lg(:,1), ...                   % F
    model.r0Rg(:,1), model.r0Hg(:,1)] ...                   % K
                        = k(model.q0, 1, model, params);    % M
    model.mode(1)       = params.mode;
    model.r0CoMg(:,1)   = rCoM(model.q0,1,model,params);

    % +-+-+-+-+-+-+-+-+-+-+-+
    ROBOT_FRAME = figure(1);
        hold on
        grid on
        axis equal
        set(gca,'Color','#CCCCCC');
        title("3D Model - ZMP Walking",'FontSize',12);
        xlabel('{\bfZ} (metres)');
        ylabel('{\bfX} (metres)');
        zlabel('{\bfY} (metres)');
        view(-165,50);
        [~] = plotRobot(1,model,params);
    % +-+-+-+-+-+-+-+-+-+-+-+

%% Generate Trajectory
   model.glbTrj = trajGen_sin(model.tspan, ...     % Time Span
                              model.xe(1:3,1)/2);  % Init Position
        [~] = plotSteps(model);

%% STEPPING
    % Helper Functions
    gradFUNC = @(A,B) (B(2) - A(2)) ...
                     /(B(1) - A(1));  % Gradient -> ∇
    % Initialise variables
    stpLngth = params.StepLength;     % Step Size:   m
    Q        = model.glbTrj;          %         Q:  [x y z]ᵀ
    Qstep    = zeros(6,length(model.glbTrj)); % Qstep:  [x 0 z]ᵀ
    r        = params.HipWidth/2;     % Radius of Circle
    STEP     = params.mode;           % DEFINE MODE:  1 RIGHT Step 
                                      %              -1 LEFT  Step
    accuDist = 0;                     % Accumulated Distance
    model.pREF(:,1) = Q([1 3],1);     % Last Foot Step -> Traj Start Point
    A        = model.glbTrj(:,1);     % A = [x₁ y₁ z₁]ᵀ
    t_begin  = 2;                     % Index of Step Beginning
    model.TBE = eye(4);               % HomoTrans: Base -> End Effector
    for i=2:length(model.tspan)
        tic
        accuDist = accuDist + norm(Q(:,i-1) - Q(:,i));
        if accuDist > stpLngth || i == length(Q)
            % TAKING STEP
            t_end         = i;       % Index of Step Ending
            params.mode   = STEP;    % Mode
            j             = t_begin; % `j` runs the step

            B = model.glbTrj(:,i);
            M = gradFUNC(A([1 3]),B([1 3]));
            % Right (+) & Left (-)
            model.pREF(:,i) = B([1 3]) + STEP*[M*r*sqrt(1/(1+M^2)); ...
                                                -r*sqrt(1/(1+M^2))];

            
            if params.mode == 1
                STEPPING = model.r0Rg(:,j-1);
            elseif params.mode == -1
                STEPPING = model.r0Lg(:,j-1);
            end
            % GENERATE STEP TRAJECTORY
            Qstep(:,t_begin:t_end) = trajGenStep(STEPPING, ...
                                       model.pREF(:,t_end), ...
                                       t_begin:t_end, ...
                                       model,params);

            for j=t_begin:t_end
                jn = j - 1;
                model.mode(j) = params.mode;
                model.q(:,j)  = k_Inv(model.q(:,jn), Qstep(:,j), j, model, params);
               [model.xe(:,j),   model.r0Lg(:,j),  ...      % F
                model.r0Rg(:,j), model.r0Hg(:,j)]  ...      % K 
                    = k(model.q(:,j), j-1, model, params);  % M
                model.r0CoMg(:,j) = rCoM(model.q(:,j),j,model,params);
            end

            % CLEAN UP
            STEP      = STEP * -1;
            accuDist  = 0;
            A         = B;
            t_begin   = t_end - 1;
            toc
        else
            model.pREF(:,i) = model.pREF(:,i-1);
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
    view(145,30);
    [~] = plotRobot(1,model,params);

    for i=1:length(model.tspan)
        cla(ROBOT_FRAME)
        params.mode = model.mode(i);
        [~] = plotRobot(i,model,params);
        IMAGE(i) = getframe(gcf);
    end

%% VIDEO
videoWriterObj           = VideoWriter('3D_Step.mp4','MPEG-4');
videoWriterObj.FrameRate = params.framerate; % 15sec video
open(videoWriterObj);                        
for i=1:length(IMAGE)
    frame = IMAGE(i);   % Convert from an Image to a Frame
    writeVideo(videoWriterObj, frame);
end
close(videoWriterObj);
