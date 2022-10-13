clear all
close all
clc 

%% Video & Time Parameters
    params.framerate  = 40;                             % FPS
    model.timestp     = params.framerate^(-1);          % Seconds
    model.tspan       = 0 : model.timestp : 20;         % [ time ]
    model.timeHrzn    = 2;                              % Seconds
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

    params.fibula       = 0.4;      % m    - Lower leg
    params.femur        = 0.4;      % m    - Upper Leg
    params.HipWidth     = 0.2;      % m    - Pelvis
    params.ServoSize    = 0.05;     % m    - Approximation/Spacing
    params.StepLength   = 0.15;     % m    - 15 cm Step Forwards
    params.StepHeight   = 0.08;     % m    - 8 cm Step Upwards

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
    model.p.pREF   = [model.p.y zeros(2,model.Nl)]; % pₓ₂REF  [REFx REFz]ᵀ
    model.p.u      = model.p.y;                     % Uₓ₂         [Ux Uz]        

%% Initial Position & Orientation
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
    model.r.xe(:,1) = k(model.r.q0, params);
    model.r.r0CoMg(:,1) = rCoM(model.r.q0,params);      % <- DUPLICATE
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
   [model.glbTrj,~,~] = ...
       trajGenGlobal(model.tspan, ...       % Time Span
                     model.r.xe(1:3,1)./2); % Init Position
        %[~] = plotSteps(model,1:length(model.tspan));
%% STEPPING
    % Helper Functions
    gradFUNC = @(A,B) (B(2) - A(2)) ...
                     /(B(1) - A(1));  % Gradient -> ∇
    % Initialise variables
    Nl       = model.Nl;              % N# INTEGER Future Indexes
    stpLngth = params.StepLength;     % Step Size:   m
    Q        = model.glbTrj;          %         Q:  [x y z]ᵀ
    Qstep    = zeros(6,length(Q)); % Qstep:  [x 0 z]ᵀ
    r        = params.HipWidth/2;     % Radius of Circle
    STEP     = params.mode;           % DEFINE MODE:  1 RIGHT Step 
                                      %              -1 LEFT  Step
    current_Dist = 0;                 % Accumulated Current Distance
    preview_Dist = 0;                 % Accumulated Preview Distance
    model.p.pREF(:,1) = model.glbTrj([1 3],1);   % Last Foot Step -> Traj Start Point
    A        = model.glbTrj(:,1);     % A = [x₁ y₁ z₁]ᵀ
    t_begin  = 1;                     % Index of Step Beginning
    model.TBE = eye(4);               % HomoTrans: Base -> End Effector
    for i=2:length(model.tspan)
        tic
        current_Dist = current_Dist + ...
            norm(model.glbTrj(:,i-1) - model.glbTrj(:,i));

        % PREVIEW FORWARD
        preview_A    = updateCoord(model.TBE, A);
        preview_STEP = STEP;
        preview_init = i + 1;
        preview_end  = i + Nl;
        preview_Dist = current_Dist;
        for p = preview_init:preview_end
            if p > length(Q)
                model.p.pREF(:,p) = model.p.pREF(:,p-1);
            else
                preview_Dist = preview_Dist + ...
                    norm(model.glbTrj(:,p-1) - model.glbTrj(:,p));

                if preview_Dist > stpLngth || p == length(Q)
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
            % TAKE STEP
            t_end           = i;       % Index of Step Ending
            params.mode     = STEP;    % Mode
            j               = t_begin; % `j` runs the step
            model.r.xe(:,j) = k(model.r.q(:,j), params); % Update Xe
            
            for u=t_begin:length(model.tspan)
                % UPDATE: Global Trajectory to End Effector Coords
                model.glbTrj(:,u) = updateCoord(model.TBE, model.glbTrj(:,u));
                % UPDATE: Pendulum Model to End Effector Coords
                tempOUT = ...
                    updateCoord(model.TBE, [model.p.y(1,u); ...
                                                         0; ...
                                            model.p.y(2,u)]);
                tempPOS = ...
                    updateCoord(model.TBE, [model.p.x(1,u); ...
                                                         0; ...
                                            model.p.x(4,u)]);
                tempVEL = ...
                    model.TBE(1:3,1:3)' * [model.p.x(2,u);
                                                        0; 
                                           model.p.x(5,u)];
                tempACC = ...
                    model.TBE(1:3,1:3)' * [model.p.x(3,u); 
                                                        0; 
                                           model.p.x(6,u)];
        
                model.p.x(:,u) = [tempPOS(1); tempVEL(1); tempACC(1);
                                  tempPOS(3); tempVEL(3); tempACC(3)];
                model.p.y(:,u) = tempOUT([1 3]);
            end

            for u=t_begin:length(model.p.pREF)
                % UPDATE: Pendulum REF to End Effector Coords
                tempREF = [model.p.pREF(1,u); 0; model.p.pREF(2,u)];
                tempREF = updateCoord(model.TBE, tempREF);
                model.p.pREF(:,u) = tempREF([1 3]);
            end

            % GENERATE STEP TRAJECTORY
            Qstep(:,t_begin:t_end) = trajGenStep(model.r.xe(:,j), ...
                                       model.p.pREF(:,t_end), ...
                                       t_begin:t_end, ...
                                       model,params);

%             % +-+-+-+-+-+-+-+-+-+-+-+
%             ROBOT_FRAME = figure(1);
%                 hold on
%                 grid on
%                 axis equal
%                 set(gca,'Color','#CCCCCC');
%                 title("3D Model - ZMP Walking",'FontSize',12);
%                 xlabel('{\bfZ} (metres)');
%                 ylabel('{\bfX} (metres)');
%                 zlabel('{\bfY} (metres)');
%                 view(-165,50);
%                 [~] = plotRobot(t_begin,model,params);
%                 [~] = plotSteps(model,t_begin:(t_end+Nl));
%                 t = t_begin:t_end;
%                 plot3(Qstep(3,t),Qstep(1,t),Qstep(2,t),'b-','LineWidth',2)
%                 pause(0.01)
%             % +-+-+-+-+-+-+-+-+-+-+-+

            for j=t_begin:t_end
                jn = j - 1;
                if jn < 1
                    jn = 1;
                end
    
                model.mode(:,j)     = [params.mode; t_begin];
                [ZMPk, CoMk, model] = LIPM3D(model,j,params);
                model.r.r0CoMg(:,j) = [CoMk(1); params.zc; CoMk(2)];
                
                xeSTAR = Qstep(:,j);
                model.r.q(:,j)  = k_Inv(model.r.q(:,jn), ...
                                        xeSTAR, j, model, params);
                [model.r.xe(:,j), model.TBE] = k(model.r.q(:,j), params);
            end

            % CLEAN UP
            current_Dist    = 0;
            STEP            = STEP * -1;
            t_begin         = t_end - 1;
            toc
        end
    end

%% Animation
    ROBOT_FRAME = figure(1);
    hold on
    grid on
    grid("minor")
    set(gca,'Color','#CCCCCC');
    title("3D Model - ZMP Walking",'FontSize',12);
    xlabel('{\bfZ} (metres)');
    ylabel('{\bfX} (metres)');
    zlabel('{\bfY} (metres)');
    view(-165,50);
    [~] = plotRobot(i,model,params);
    
    for i=1:length(model.tspan)
        params.mode = model.mode(1,i);
        cla(ROBOT_FRAME)
        CM = model.r.r0CoMg([1 3],i);
        axis([ CM(2)-0.5, CM(2)+0.5, CM(1)-0.5, CM(1)+0.5, 0.0, 1.0]);
        plot3(CM(2), CM(1), 0, 'mx','LineWidth',2,'MarkerSize',5)
        [~] = plotRobot(i,model,params);
        %[~] = plotSteps(model);
        %[~] = plotPend(i,model,params);
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
