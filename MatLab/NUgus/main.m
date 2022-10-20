clear all
close all
clc 

%% Video & Time Parameters
    params.framerate  = 25;                             % FPS
    params.timestp    = params.framerate^(-1);          % Seconds
    model.tspan       = 0 : params.timestp : 120;       % [ time ]
    model.timeHrzn    = 0;                            % Seconds
    model.Nl          = model.timeHrzn / params.timestp; % INTEGER
 % Physical Parameters - Affect CoM or FKM   |
    params.zc           = 0.40;    % m    - Height of the CoM ^         |
    params.m            = 7.4248;  % kg   - Total Mass of a NuGus       |
    params.StepLength   = 0.15;    % m    - 15 cm Step Forward          |
    params.StepHeight   = 0.07;    % m    - 7 cm Step Height           |
 % ----------------------------------------------------------------------|
 % Displacements
    params.fibula     = 0.19942;  % m    - Lower leg NUgus
    params.femur      = 0.19954;  % m    - Upper Leg NUgus
    params.HipWidth   = 0.16;     % m    - Pelvis   WAS 0.11 -> FALLOVER
    params.heel2ankle = 0.038;    % m    - Tarsal
    params.ankle2knee = [0.005669; 0.19942; -0.0002];
    params.knee2hip   = [       0; 0.19954; -0.0015];
    params.hip2waist  = [-0.06895; 0.04625; -0.0545];
    params.ServoSize  = 0.05;     % m    - Approximation/Spacing
 % Masses
    params.mass.fibula = 0.1719;                    % Paired with `tibia`
    params.mass.femur  = 0.3423;                    % Thigh Bone
    params.mass.joint  = 0.300  + 0.7;              % Knee Bone / Joints
    params.mass.pelvis = 2.9549 + 697.6 + 753.5;    % Waist
    params.mass.foot   = 0.2023 + 0.5;              % Foot

%% Model setup
 % Steps
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

    webotsMod   = [-1;  % θ₁
                    1;  % θ₂
                    1;  % θ₃
                    1;  % θ₄
                   -1;  % θ₅
                    1;  % θ₆
                   -1;  % θ₇
                    1;  % θ₈
                   -1;  % θ₉
                   -1;  % θ₁₀
                   -1;  % θ₁₁
                    1]; % θ₁₂

    model.q(:,1)      = model.q0;
    model.xe(:,1)     = k(model.q0, params);
    model.r0CoMg(:,1) = rCoM(model.q0,params);
    model.mode(1)     = params.mode;
    arms = [deg2rad(105);
            deg2rad(-10);
            deg2rad(-30);
            deg2rad(-30);
            deg2rad(10);
            deg2rad(105)];

    % +-+-+-+-+-+-+-+-+-+-+-+
    ROBOT_FRAME = figure(1);
        cla(ROBOT_FRAME)
        hold on
        grid on
        axis equal
        set(gca,'Color','#CCCCCC');
        title("3D Model - ZMP Walking",'FontSize',12);
        view(-145,50);
        xlabel('{\bfZ} (metres)');
        ylabel('{\bfX} (metres)');
        zlabel('{\bfY} (metres)');
        [~] = plotRobot(1,model,params);
    % +-+-+-+-+-+-+-+-+-+-+-+
%     for i=1:5
%         commJointValues("10.0.0.127","10013",...
%             [webotsMod.*model.q(:,1); arms./(6 - i)]...
%         );
%     end
    % +-+-+-+-+-+-+-+-+-+-+-+


%% Generate Trajectory
    model.glbTrj = ...
       trajGen_cir(model.tspan, model.xe(1:3,1)./2);
%        trajGenC(1/params.framerate, ...       % Time Span
%                 model.xe(1:3,1)./2);          % Init Position
    [~] = plotSteps(model);
%% STEPPING
    % Helper Functions
    gradFUNC = @(A,B) (B(2) - A(2)) ...
                     /(B(1) - A(1));  % Gradient -> ∇
    % Initialise variables
    stpLngth = params.StepLength;              % Step Size:   m
    Q        = model.glbTrj;                   % Q:         [x y z]ᵀ
    Qstep    = zeros(6,length(model.glbTrj));  % Qstep:     [xe]
    r        = (params.HipWidth/2) + 0.02;     % Radius of Circle + shim
    STEP     = params.mode;           % DEFINE MODE:  1 RIGHT Step 
                                      %              -1 LEFT  Step
    accuDist        = 0;                 % Accumulated Distance
    model.pREF(:,1) = Q([1 3],1);        % Last Foot Step -> TrajStartPoint
    A               = model.glbTrj(:,1); % A = [x₁ y₁ z₁]ᵀ
    t_begin         = 2;                 % Index of Step Beginning
    model.TBE       = eye(4);            % HomoTrans: Base -> End Effector
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

            for j=t_begin:t_end
                jn = j - 1;
                model.mode(:,j)     = params.mode;
                xeSTAR = Qstep(:,j);
                model.q(:,j)  = k_Inv(model.q(:,jn), ...
                                        xeSTAR, j, model, params);
                [model.xe(:,j), model.TBE] = k(model.q(:,j), params);
                DEBUG(j,t_begin,t_end,Qstep,model,params);
                % Send qᵢ
%                 commJointValues("10.0.0.127","10013",...
%                     [webotsMod.*model.q(:,j); arms]...
%                 );
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

%% RESEND q(t)

% for i=1:5
%     commJointValues("10.0.0.127","10013",...
%         [webotsMod.*model.q(:,1); arms]...
%     );
% end
% for i=1:length(model.tspan)
%         commJointValues("10.0.0.127","10013",...
%         [webotsMod.*model.q(:,i); arms./(6 - i)]...
%     );
% end

%% VIDEO
videoWriterObj           = VideoWriter('3D_Step.mp4','MPEG-4');
videoWriterObj.FrameRate = params.framerate; % 15sec video
open(videoWriterObj);                        
for i=1:length(IMAGE)
    frame = IMAGE(i);   % Convert from an Image to a Frame
    writeVideo(videoWriterObj, frame);
end
close(videoWriterObj);
