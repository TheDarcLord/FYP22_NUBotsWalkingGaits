clear all
close all
clc 

%% Video & Time Parameters
    params.framerate  = 25;                             % FPS
    params.timestp    = params.framerate^(-1);          % Seconds
    model.tspan       = 0 : params.timestp : 200;       % [ time ]
 % Physical Parameters - Affect CoM or FKM
    params.kx        = 0;        % These affect the plane to which    |
    params.ky        = 0;        % ... the CoM is constrained         |
    params.zc        = 0.4564;   % m    - Height of the CoM ^         |
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
    params.mass.foot   = 0.0;   % foot
    params.mass.fibula = 1.5;    % Paired with `tibia`
    params.mass.femur  = 1.5;    % Thigh Bone
    params.mass.joint  = 0.5;    % Knee Bone / Joints
    params.mass.pelvis = 1.5;    % Waist

%% Model setup
 % Steps
    % Stepping mode... Array!? ... ?
    model.mode     = zeros(1,length(model.tspan));
    params.mode    = -1;    % LEFT  FIXED
    %                 0;    % BOTH  FIXED
    %                 1;    % RIGHT FIXED
 % Robot
    model.q      = zeros(12,length(model.tspan)); % q   [θ₁θ₂θ₃ ...]ᵀ
    model.xe     = zeros(6,length(model.tspan));  % xe      [XYZϕθΨ]
    model.r0CoMg = zeros(3,length(model.tspan));  % r0CoMg  [XYZ]ᵀ
    model.pREF   = zeros(2,length(model.tspan));  % stpREF  [XZ]ᵀ

%% Initial Position & Orientation
    model.q0        = [  0;    % θ₁    
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
    model.q(:,1) = model.q0;
    model.xe(:,1) = k(model.q0, params);
    model.r.r0CoMg(:,1) = rCoM(model.q0,params);      % <- DUPLICATE
    model.mode(1) = params.mode;
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
%     model.glbTrj = trajGenC(params.timestp, model.xe(1:3,1)./2);
%     model.glbTrj = trajGen_cir(model.tspan, model.xe(1:3,1)./2);
    model.glbTrj = trajGen_sin(model.tspan, model.xe(1:3,1)./2);
        [~] = plotSteps(model,1:length(model.tspan));

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
            t_end           = i;       % Index of Step Ending
            params.mode     = STEP;    % Mode
            j               = t_begin; % `j` runs the step
            model.xe(:,j) = k(model.q(:,j), params); % Update Xe
            
            A = updateCoord(model.TBE, A);

            for u=t_begin:length(model.tspan)
                % UPDATE: Global Trajectory to End Effector Coords
                model.glbTrj(:,u) = ...
                    updateCoord(model.TBE, model.glbTrj(:,u));
                tmpREF = [model.pREF(1,u); 0; model.pREF(2,u)];
                tmpREF = ...
                    updateCoord(model.TBE, tmpREF);
                model.pREF(:,u) = tmpREF([1 3]);
            end

            B = model.glbTrj(:,i);
            M = gradFUNC(A([1 3]),B([1 3]));
            % Right (+) & Left (-)
            model.pREF(:,i) = B([1 3]) + STEP*[M*r*sqrt(1/(1+M^2)); ...
                                                -r*sqrt(1/(1+M^2))];

            % GENERATE STEP TRAJECTORY
            Qstep(:,t_begin:t_end) = trajGenStep(model.xe(:,j), ...
                                       model.pREF(:,t_end), ...
                                       t_begin:t_end, ...
                                       model,params);
%             ROBOT_FRAME = figure(1);
%                 cla(ROBOT_FRAME)
%                 hold on
%                 grid on
%                 set(gca,'Color','#CCCCCC');
%                 title("3D Model - ZMP Walking",'FontSize',12);
%                 xlabel('{\bfZ} (metres)');
%                 ylabel('{\bfX} (metres)');
%                 zlabel('{\bfY} (metres)');
%                 view(-165,50);
%                 plot3(Qstep(3,t_begin:t_end), ...
%                       Qstep(1,t_begin:t_end), ...
%                       Qstep(2,t_begin:t_end),'b-','LineWidth',2);
%                 [~] = plotRobot(t_begin-1,model,params);
%                 [~] = plotSteps(model,t_begin:t_end);
            for j=t_begin:t_end
                jn = j - 1;
                model.mode(j)     = params.mode;
                xeSTAR = Qstep(:,j);
                model.q(:,j)  = k_Inv(model.q(:,jn), ...
                                      xeSTAR, j, model, params);
                [model.xe(:,j), model.TBE] = k(model.q(:,j), params);
%                 % DEBUG           
%                 ROBOT_FRAME = figure(1);
%                 cla(ROBOT_FRAME)
%                 hold on
%                 grid on
%                 set(gca,'Color','#CCCCCC');
%                 title("3D Model - ZMP Walking",'FontSize',12);
%                 xlabel('{\bfZ} (metres)');
%                 ylabel('{\bfX} (metres)');
%                 zlabel('{\bfY} (metres)');
%                 view(-165,50);
%                 [~] = plotRobot(j,model,params);
%                 [~] = plotSteps(model,t_begin:t_end);
%                 plot3(Qstep(3,t_begin:t_end), ...
%                       Qstep(1,t_begin:t_end), ...
%                       Qstep(2,t_begin:t_end),'b-','LineWidth',2);
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
    view(-165,45);

    for i=1:length(model.tspan)
        cla(ROBOT_FRAME)
        params.mode = model.mode(i);
        [~] = plotRobot(i,model,params);
        %[~] = plotSteps(model,1:i);
        
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
