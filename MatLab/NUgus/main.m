clear all
close all
clc 

%% Video & Time Parameters
    params.framerate  = 40;                             % FPS
    model.timestp     = params.framerate^(-1);          % Seconds
    model.tspan       = 0 : model.timestp : 20;         % [ time ]
    model.timeHrzn    = 0;                            % Seconds
    model.Nl          = model.timeHrzn / model.timestp; % INTEGER
 % Physical Parameters - Affect CoM or FKM   |
    params.zc           = 0.328;    % m    - Height of the CoM ^         |
    params.m            = 7.4248;   % kg   - Total Mass of a NuGus       |
    params.StepLength   = 0.15;     % m    - 15 cm Step Forward          |
    params.StepHeight   = 0.05;     % m    - 10 cm Step Height           |
 % ----------------------------------------------------------------------|
 % Displacements
    params.fibula     = 0.19942;  % m    - Lower leg NUgus
    params.femur      = 0.19954;  % m    - Upper Leg NUgus
    params.HipWidth   = 0.11;     % m    - Pelvis
    params.heel2ankle = 0.038;    % m    - Tarsal
    params.ankle2knee = [0.005669; 0.19942; -0.0002];
    params.knee2hip   = [       0; 0.19954; -0.0015];
    params.hip2waist  = [-0.06895; 0.04625; -0.0545];
    params.ServoSize  = 0.05;     % m    - Approximation/Spacing
 % Masses
    params.mass.fibula = 0.1298;        % Paired with `tibia`
    params.mass.femur  = 0.3423;        % Thigh Bone
    params.mass.joint  = 0.2000;        % Knee Bone / Joints
    params.mass.pelvis = 2.9549;    % Waist
    params.mass.foot   = 0.2023;        % Foot

%% Model setup
 % Steps
    % Stepping mode... Array!? ... ?
    model.mode     = zeros(1,length(model.tspan));
    params.mode    = -1;       % LEFT  FIXED - FKM T16
    %                 0;       % BOTH  FIXED - FKM T1H T6H
    %                 1;       % RIGHT FIXED - FKM T61
 % Robot
    model.q      = zeros(12,length(model.tspan)); % q        [θ₁θ₂θ₃ ...]ᵀ
    model.xe     = zeros(6,length(model.tspan));  % xe           [XYZϕθΨ]ᵀ
    model.r0CoMg = zeros(3,length(model.tspan));  % r0CoMg          [XYZ]ᵀ
    model.pREF   = zeros(2,length(model.tspan));  % pREF [REFx REFy REFz]ᵀ      

%% Initial Position & Orientation
    model.q0      = [0;    % θ₁    
                -pi/10;    % θ₂    ->  2D θ₁ Ankle
               2*pi/10;    % θ₃    ->  2D θ₂ Knee
                -pi/10;    % θ₄    ->  2D θ₃ Hip
                     0;    % θ₅
                     0;    % θ₆
                     0;    % θ₇
                     0;    % θ₈
                 pi/10;    % θ₉    ->  2D θ₄ Hip
              -2*pi/10;    % θ₁₀   ->  2D θ₅ Knee
                 pi/10;    % θ₁₁   ->  2D θ₆ Ankle
                     0];   % θ₁₂

    model.q(:,1)      = model.q0;
    model.xe(:,1)     = k(model.q0, params);
    model.r0CoMg(:,1) = rCoM(model.q0,params);

    % +-+-+-+-+-+-+-+-+-+-+-+
    ROBOT_FRAME = figure(1);
        hold on
        grid on
        set(gca,'Color','#CCCCCC');
        title("3D Model - ZMP Walking",'FontSize',12);
        xlabel('{\bfZ} (metres)');
        ylabel('{\bfX} (metres)');
        zlabel('{\bfY} (metres)');
        view(-165,50);
        [~] = plotRobot(1,model,params);
    % +-+-+-+-+-+-+-+-+-+-+-+

%% Generate Trajectory
   [model.glbTrj,~,~] = ...
       trajGenGlobal(model.tspan, ...       % Time Span
                     model.xe(1:3,1)./2); % Init Position

%% STEPPING
    % Helper Functions
    gradFUNC = @(A,B) (B(2) - A(2)) ...
                     /(B(1) - A(1));  % Gradient -> ∇
    % Initialise variables
    Nl       = model.Nl;              % N# INTEGER Future Indexes
    stpLngth = params.StepLength;     % Step Size:   m
    Q        = model.glbTrj;          %         Q:  [x y z]ᵀ
    Qstep    = zeros(6,length(model.glbTrj)); % Qstep:  [x 0 z]ᵀ
    r        = params.HipWidth/2;     % Radius of Circle
    STEP     = params.mode;           % DEFINE MODE:  1 RIGHT Step 
                                      %              -1 LEFT  Step
    accuDist = 0;                     % Accumulated Distance
    model.pREF(:,1) = Q([1 3],1);     % Last Foot Step -> Traj Start Point
    A        = model.glbTrj(:,1);     % A = [x₁ y₁ z₁]ᵀ
    t_begin  = 1;                     % Index of Step Beginning
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
%             hold on
%             grid on
%             set(gca,'Color','#CCCCCC');
%             title("3D Model - ZMP Walking",'FontSize',12);
%             xlabel('{\bfZ} (metres)');
%             ylabel('{\bfX} (metres)');
%             zlabel('{\bfY} (metres)');
%             view(-165,50);
%             [~] = plotSteps(model);
%             plot3(Qstep(3,:),Qstep(1,:),Qstep(2,:),'b-','LineWidth',2)

            for j=t_begin:t_end
                jn = j - 1;
                if jn < 1
                    jn = 1;
                end
    
                model.mode(:,j)     = params.mode;
                xeSTAR = Qstep(:,j);
                model.q(:,j)  = k_Inv(model.q(:,jn), ...
                                        xeSTAR, j, model, params);
                [model.xe(:,j), model.TBE] = k(model.q(:,j), params);
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
        [~] = plotSteps(model);
        
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
