clear all
close all
clc 

%% Video & Time Parameters
    params.framerate  = 40;                             % FPS
    model.timestp     = params.framerate^(-1);          % Seconds
    model.tspan       = 0 : model.timestp : 20;         % [ time ]
    model.timeHrzn    = 1.5;                            % Seconds
    model.Nl          = model.timeHrzn / model.timestp; % INTEGER
 % Weights for controller `Performance Index`
    % Design of an optimal controller for a discrete-time system subject
    % to previewable demand
    %   1985 - KATAYAMA et al.
    %      ∞     
    % Jᵤ = Σ [ e(i)ᵀ⋅Qₑ⋅e(i) + Δx(i)ᵀ⋅Qₓ⋅Δx(i) + Δu(i)ᵀ⋅R⋅u(i) ]
    %     i=k
    % where:
    %        e(i): ZMPₓ(i) - Yₓ     aka Tracking Error 
    %       Δx(i): x(i) - x(i-1)    aka Incremental State Vector
    %       Δu(i): u(i) - u(i-1)    aka Incremental Control Vector
    params.weights.Qe   =        eye(2,2);     
    params.weights.Qx   = 0    * eye(6,6);
    params.weights.R    = 1e-3 * eye(2,2);
 % Physical Parameters - Affect CoM or FKM
    params.kx        = 0;        % These affect the plane to which    |
    params.ky        = 0;        % ... the CoM is constrained         |
    params.zc        = 0.25;   % m    - Height of the CoM ^         |
    params.g         = 9.81;     % ms⁻² - Acceleration due to Gravity |
    params.m         = 7.4248;   % kg   - Total Mass of a NuGus       |
 % -------------------------------------------------------------------|
    params.fibula     = 0.19942;  % m    - Lower leg NUgus
    params.femur      = 0.19954;  % m    - Upper Leg NUgus
    params.HipWidth   = 0.11;     % m    - Pelvis
    params.heel2ankle = 0.038;    % m    - Tarsal
    params.ankle2knee = [0.005669; 0.19942; -0.0002];
    params.knee2hip   = [       0; 0.19954; -0.0015];
    params.hip2waist  = [-0.06895; 0.04625; -0.0545];
    params.ServoSize  = 0.05;     % m    - Approximation/Spacing
    params.StepSize   = 0.10;     % m    - 15 cm Step forward
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
    model.r.r0Lg(:,1) = [0; 0;  params.HipWidth/2;0;0;0];
    model.r.r0Rg(:,1) = [0; 0; -params.HipWidth/2;0;0;0];
    model.r.q0        = [0;    % θ₁    
                    -pi/6;    % θ₂    ->  2D θ₁ Ankle
                   2*pi/6;    % θ₃    ->  2D θ₂ Knee
                    -pi/6;    % θ₄    ->  2D θ₃ Hip
                         0;    % θ₅
                         0;    % θ₆
                         0;    % θ₇
                         0;    % θ₈
                     pi/6;    % θ₉    ->  2D θ₄ Hip
                  -2*pi/6;    % θ₁₀   ->  2D θ₅ Knee
                     pi/6;    % θ₁₁   ->  2D θ₆ Ankle
                         0];   % θ₁₂
    model.r.q(:,1) = model.r.q0;
   [model.r.xe(:,1),   model.r.r0Lg(:,1), ...% F
    model.r.r0Rg(:,1), model.r.r0Hg(:,1)] ...% K
        = kNU(model.r.q0, 1, model, params);   % M

    model.r.r0CoMg(:,1) = rCoM(model.r.q0,1,model,params);
    model.p.x(:,1) = [model.r.r0CoMg(1,1); 0; 0;  % Position X
                      model.r.r0CoMg(3,1); 0; 0]; % Position Z

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
   [model.glbTrj,~,~] = trajGenGlobal(model.tspan, ...     % Time Span
                                      model.r.r0Rg(1:3,1));% Init Position

%% Generate ZMP Reference
   [model.p.pREF, model.p.sTM] = pREF(model,params);

%% STEPPING
    Q   = [];
    t_p = 1;
    for i=1:length(model.p.sTM)-1
        tic
        t_c = model.p.sTM(1,i);         % Time Index CURRENT
        t_n = model.p.sTM(1,i+1);       % Time Index NEXT
        params.mode = model.p.sTM(2,i); % Mode
        Q = [Q ...
             trajGenStep(model.p.pREF(:,t_n),   ...
                         t_c:(t_n-1),           ...
                         t_p,                   ...
                         model,params)];
        
        for j=t_c:(t_n-1)
            
            jn = j - 1;
            if jn < 1
                jn = 1;
            end

            model.mode(j)       = params.mode;
            [ZMPk, CoMk, model] = LIPM3D(model,j,params);
            model.r.r0CoMg(:,j) = [CoMk(1); params.zc; CoMk(2)];
            
            model.r.xe(:,j)   = [Q(:,j)];
            model.r.q(:,j)    = k_Inv(model.r.q(:,jn), model.r.xe(:,j), j, model, params);
           [model.r.xe(:,j),   model.r.r0Lg(:,j),  ...  % F
            model.r.r0Rg(:,j), model.r.r0Hg(:,j)]  ...  % K 
                = kNU(model.r.q(:,j), jn, model, params); % M
        end
        t_p = t_n - 1;
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
    for i=1:length(model.tspan)-19
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
videoWriterObj.FrameRate = params.framerate; % 15sec video
open(videoWriterObj);                        
for i=1:length(IMAGE)
    frame = IMAGE(i);   % Convert from an Image to a Frame
    writeVideo(videoWriterObj, frame);
end
close(videoWriterObj);
