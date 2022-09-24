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
    params.zc        = 0.4564;   % m    - Height of the CoM ^         |
    params.g         = 9.81;     % ms⁻² - Acceleration due to Gravity |
    params.m         = 7.4248;   % kg   - Total Mass of a NuGus       |
 % -------------------------------------------------------------------|
    params.fibula    = 0.4;      % m    - Lower leg
    params.femur     = 0.4;      % m    - Upper Leg
    params.HipWidth  = 0.2;      % m    - Pelvis
    params.ServoSize = 0.05;     % m    - Approximation/Spacing
    params.StepSize  = 0.15;     % m    - 15 cm Step forward
 % Masses
    params.mass.foot   = 0.25;   % foot
    params.mass.fibula = 1.5;    % Paired with `tibia`
    params.mass.femur  = 1.5;    % Thigh Bone
    params.mass.joint  = 0.5;    % Knee Bone / Joints
    params.mass.pelvis = 1.5;    % Waist

%% Model setup
 % Steps
    % Stepping mode... Array!? ... ?
    model.mode     = zeros(2,length(model.tspan));
    params.mode    = -1;    % LEFT  FIXED
    %                 0;    % BOTH  FIXED
    %                 1;    % RIGHT FIXED
 % Robot
    model.r.q      = zeros(12,length(model.tspan)); % q   [θ₁θ₂θ₃ ...]ᵀ
    model.r.xe     = zeros(6,length(model.tspan));  % xe      [XYZϕθΨ]
    model.r.r0CoMg = zeros(3,length(model.tspan));  % r0CoMg  [XYZ]ᵀ
 % Pendulum
    model.p.x      = zeros(6,length(model.tspan));  % Xcom      [x x' x"]ᵀ
    model.p.y      = zeros(2,length(model.tspan));  % pₓ₂     [ZMPx ZMPz]ᵀ
    model.p.pREF   = model.p.y;                     % pₓ₂REF  [REFx REFz]ᵀ
    model.p.u      = model.p.y;                     % Uₓ₂         [Ux Uz]        

% ---> Initial Position & Orientation
    model.r.q0        = [0;    % θ₁    
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
    model.r.q(:,1) = model.r.q0;
   [model.r.xe(:,1), TAE] = k(model.r.q0, params);
    model.r.r0CoMg(:,1) = rCoM(model.r.q0,params);      % <- DUPLICATE
    model.p.x(:,1) = [model.r.r0CoMg(1,1); 0; 0;  % Position X
                      model.r.r0CoMg(3,1); 0; 0]; % Position Z

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

% ---> Generate Trajectory
    midpoint = TAE(1:3,4)./2;
   [model.glbTrj,~,~] = trajGenGlobal(model.tspan, ...  % Time Span
                                      midpoint);        % Init Position
% ---> Generate ZMP Reference
   [model.p.pREF, model.p.sTM] = pREF(model,params);
        [~] = plotSteps(1,model);
%% STEPPING
    Q   = [];
    model.TBE = eye(4);
    j   = 1;
    for i=1:length(model.p.sTM)-1
        tic
        t_c = model.p.sTM(1,i);         % Time Index CURRENT
        t_n = model.p.sTM(1,i+1);       % Time Index NEXT
        params.mode = model.p.sTM(2,i); % Mode

        model.r.xe(:,j) = k(model.r.q(:,j), params);    % Update Xe

        % UPDATE TRAJECTORY TO END EFFECTOR COORDS
        for u=t_c:length(model.glbTrj)
            model.glbTrj(:,u) = ...
                (model.TBE(1:3,1:3)' * (model.glbTrj(:,u) - model.TBE(1:3,4)));
                % Rbe' * ( r_traj_base - r_endEff_base )
            %model.
        end
        
        % UPDATE PENDULUM REF TO END EFFECTOR COORDS
        for u=t_c-1:length(model.p.pREF)
            if u < 1
                u = 1;
            end
            tempREF = [model.p.pREF(1,u); 0; model.p.pREF(2,u)];
            tempREF = ...
                (model.TBE(1:3,1:3)' * (tempREF - model.TBE(1:3,4)));
                % Rbe' * ( r_traj_base - r_endEff_base )
            model.p.pREF(:,u) = tempREF([1 3]);
        end

        % UPDATE PENDULUM REF TO END EFFECTOR COORDS
        for u=t_c-1:length(model.p.x)
            if u < 1
                u = 1;
            end
            tempPOS = [model.p.x(1,u); 0; model.p.x(4,u)];
            tempVEL = [model.p.x(2,u); 0; model.p.x(5,u)];
            tempACC = [model.p.x(3,u); 0; model.p.x(6,u)];
            tempOUT = [model.p.y(1,u); 0; model.p.y(2,u)];
            tempOUT = ...
                (model.TBE(1:3,1:3)' * (tempOUT - model.TBE(1:3,4)));
                % Rbe' * ( r_traj_base - r_endEff_base )
            tempPOS = ...
                (model.TBE(1:3,1:3)' * (tempPOS - model.TBE(1:3,4)));
                % Rbe' * ( r_traj_base - r_endEff_base )
            tempVEL = ...
                model.TBE(1:3,1:3)' * (tempVEL);
                % Rbe' * ( r_traj_base - r_endEff_base )
            tempACC = ...
                model.TBE(1:3,1:3)' * (tempACC);
                % Rbe' * ( r_traj_base - r_endEff_base )

            model.p.x(:,u) = [tempPOS(1); tempVEL(1); tempACC(1);
                              tempPOS(3); tempVEL(3); tempACC(3)];
            model.p.y(:,u) = tempOUT([1 3]);
        end

        Q = [Q ...
             trajGenStep(model.r.xe(:,j), ...
                       model.p.pREF(:,t_n), ...
                               t_c:(t_n-1), ...
                                    model);];
        
        for j=t_c:(t_n-1)
            jn = j - 1;
            if jn < 1
                jn = 1;
            end

            model.mode(:,j)       = [params.mode; t_c];
            [ZMPk, CoMk, model] = LIPM3D(model,j,params);
            model.r.r0CoMg(:,j) = [CoMk(1); params.zc; CoMk(2)];
            
            xeSTAR = Q(:,j);
            model.r.q(:,j)  = k_Inv(model.r.q(:,jn), ...
                                    xeSTAR, j, model, params);
            [model.r.xe(:,j), model.TBE] = k(model.r.q(:,j), params); % M
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
    view(-165,50);
    [~] = plotRobot(i,model,params);
    
    for i=1:length(Q)
        params.mode = model.mode(1,i);
        cla(ROBOT_FRAME)
        CM = model.r.r0CoMg([1 3],i);
        axis([ CM(2)-1, CM(2)+1, CM(1)-1, CM(1)+1, 0.0, 1.0]);
        [~] = plotRobot(i,model,params);
        [~] = plotSteps(model.mode(2,i),model);
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
