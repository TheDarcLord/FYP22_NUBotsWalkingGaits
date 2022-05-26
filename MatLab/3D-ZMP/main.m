clear all
close all
clc 

tic % START TIMING

%% Video/Time Parameters
    params.framerate = 100;                                   % FPS
    model.timestp    = params.framerate^(-1);                % Seconds
    model.tspan      = 0 : model.timestp : 12;               % [ time ]
    model.timeHrzn   = 1.5;                                  % Seconds
    model.Nl         = model.timeHrzn / model.timestp;      % INTEGER
 % Weights for controller `Performance Index`
    % Design of an optimal controller for a discrete-time system subject
    % to previewable demand
    %   1985 - KATAYAMA et al.
    %       ∞     
    %% Ju = Σ [ e(i)ᵀ⋅Qₑ⋅e(i) + Δx(i)ᵀ⋅Qₓ⋅Δx(i) + Δu(i)ᵀ⋅R⋅u(i) ]
    %      i=k
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
    params.zc        = 0.3148;   % m    - Height of the CoM ^         |
    params.g         = 9.81;     % ms⁻² - Acceleration due to Gravity |
    params.m         = 7.4248;   % kg   - Total Mass of a NuGus       |
 % -------------------------------------------------------------------|
    params.fibula    = 0.22;     % m    - Lower leg
    params.femur     = 0.22;     % m    - Upper Leg
    params.HipWidth  = 0.15;     % m    - Pelvis
    params.ServoSize = 0.05;     % m    - Approximation/Spacing
    params.StepSize  = 0.1;      % m    - 10 cm Step forward
 % Masses
    params.mass.fibula = 1.5;    % Paired with `tibia`
    params.mass.femur  = 1.5;    % Thigh Bone
    params.mass.joint  = 0.5;    % Knee Bone / Joints
    params.mass.pelvis = 1.5;    % Waist

 % Stepping mode... Array!?
    params.mode         = -1;          % LEFT  FIXED - FKM T16
    %                      0;          % BOTH  FIXED - FKM T1H T6H
    %                      1;          % RIGHT FIXED - FKM T61

%% Model setup
 % Robot
    model.r.q      = zeros(12,length(model.tspan)); % q     [θ₁θ₂θ₃ ...]ᵀ
    model.r.xe     = zeros(6,length(model.tspan));  % xe    [XYZϕθΨ]ᵀ
    model.r.r0Lg   = zeros(3,length(model.tspan));  % A0EL     [XYZ]ᵀ
    model.r.r0Rg   = model.r.r0Lg;                  % A0ER     [XYZ]ᵀ
    model.r.r0Hg   = model.r.r0Lg;                  % A0H      [XYZ]ᵀ
    model.r.r0CoMg = model.r.r0Lg;                  % r0CoMg   [XYZ]ᵀ
 % Pendulum
    model.p.x      = zeros(6,length(model.tspan));  % Xcom      [x x' x"]ᵀ
    model.p.y      = zeros(2,length(model.tspan));  % pₓᵧ     [ZMPx ZMPy]ᵀ
    model.p.pREF   = model.p.y;                     % pₓᵧREF  [REFx REFy]ᵀ
    model.p.u      = model.p.y;                     % Uₓᵧ         [Ux Uy]        

%% Initial Position & Orientation
model.r.q0 = [      0;     % θ₁    
                -pi/24;     % θ₂    ->  2D θ₁ Ankle
               2*pi/24;     % θ₃    ->  2D θ₂ Knee
                -pi/24;     % θ₄    ->  2D θ₃ Hip
                    0;     % θ₅
                    0;     % θ₆
                    0;     % θ₇
                    0;     % θ₈
                 pi/24;     % θ₉    ->  2D θ₄ Hip
              -2*pi/24;     % θ₁₀   ->  2D θ₅ Knee
                 pi/24;     % θ₁₁   ->  2D θ₆ Ankle
                    0];    % θ₁₂

    model.r.q(:,1)            = model.r.q0;
   [model.r.xe(:,1), HTs]     = k(model.r.q0, 1, model, params);
    
    model.r.r0Rg(:,1)         = HTs.A0ER(1:3,4);
    model.r.r0Lg(:,1)         = HTs.A0EL(1:3,4);
    model.r.r0Hg(:,1)         = HTs.A0H(1:3,4);
    model.r.r0CoMg(:,1)       = rCoM(model.r.q0,1,model,params);

%% LOOP
% Robot    - Foot Trajectory Generation
    params.mode  = -1;
    [Q,~,~,STEP] = trajectoryGeneration(1:601, 1, model, params);

% Pendulum - CoM Trajectory
    % Initial Condidtions
    ZMP0 = STEP(:,1);
    ZMP1 = STEP(:,2);
    model.p.x(:,1) = [model.r.r0CoMg(1,1); 0; 0;  % Position X
                      model.r.r0CoMg(3,1); 0; 0]; % Position Z
    ZMPtim = model.tspan(301);
    % Load ZMPₓ Reference
    model.p.pREF(:,1:601) = pREF(model.tspan(1:601), ZMP0, ZMP1, ZMPtim);
    % Simulation Loop
    for i=1:601
        [ZMPk, CoMk, model] = LIPM3D(model,i,ZMP0,ZMP1,ZMPtim,params);
        model.r.r0CoMg(:,i) = [CoMk(1); params.zc; CoMk(2)];
    end

for i=2:601
    model.r.xe(:,i)   = [Q(:,i); zeros(3,1)];
    model.r.q(:,i)    = k_Inv(model.r.q(:,i-1), model.r.xe(:,i), i, model, params);
    [~, HTs]          = k(model.r.q(:,i), i-1, model, params);
    model.r.r0Lg(:,i) = HTs.A0EL(1:3,4);
    model.r.r0Rg(:,i) = HTs.A0ER(1:3,4);
    model.r.r0Hg(:,i) = HTs.A0H(1:3,4);
end

% Robot    - Foot Trajectory Generation
    params.mode = 1;
    [Q1,~,~,STEP] = trajectoryGeneration(602:length(model.tspan), 601, model, params);
    Q = [Q Q1];
% Pendulum - CoM Trajectory
    % Initial Condidtions
    ZMP0 = STEP(:,1);
    ZMP1 = STEP(:,2);
    ZMPtim = model.tspan(901);
    % Load ZMPₓ Reference
    model.p.pREF(:,602:end) = pREF(model.tspan(602:end), ZMP0, ZMP1, ZMPtim);
    % Simulation Loop
    for i=601:length(model.tspan)
        [ZMPk, CoMk, model] = LIPM3D(model,i,ZMP0,ZMP1,ZMPtim,params);
        model.r.r0CoMg(:,i) = [CoMk(1); params.zc; CoMk(2)];
    end

for i=602:length(model.tspan)
    model.r.xe(:,i)   = [Q(:,i); zeros(3,1)];
    model.r.q(:,i)    = k_Inv(model.r.q(:,i-1), model.r.xe(:,i), i, model, params);
    [~, HTs]          = k(model.r.q(:,i), i-1, model, params);
    model.r.r0Lg(:,i) = HTs.A0EL(1:3,4);
    model.r.r0Rg(:,i) = HTs.A0ER(1:3,4);
    model.r.r0Hg(:,i) = HTs.A0H(1:3,4);
end

toc % FINISH TIMING
%%
% figure('Name','Joint Variables, q(t)')
%     subplot(1,2,1)
%         hold on
%         plot(model.tspan,model.r.q(1,:),'r-','LineWidth',2);
%         plot(model.tspan,model.r.q(2,:),'g-','LineWidth',2);
%         plot(model.tspan,model.r.q(3,:),'b-','LineWidth',2);
%         plot(model.tspan,model.r.q(4,:),'c-','LineWidth',2);
%         plot(model.tspan,model.r.q(5,:),'y-','LineWidth',2);
%         plot(model.tspan,model.r.q(6,:),'m-','LineWidth',2);
%         set(gca,'Color','#CCCCCC');
%         xlabel('Time (t) ({\itSeconds})','FontWeight','bold');
%         ylabel('qθ_{1-6} ({\itRadians})','FontWeight','bold');
%         title('Joint Variables: {\itθ}_{1-6}({\itt})','FontSize',12);
%         legend('θ₁','θ₂','θ₃', 'θ₄','θ₅','θ₆');
%     subplot(1,2,2)
%         hold on
%         plot(model.tspan,model.r.q(7,:),'m-','LineWidth',2);
%         plot(model.tspan,model.r.q(8,:),'y-','LineWidth',2);
%         plot(model.tspan,model.r.q(9,:),'c-','LineWidth',2);
%         plot(model.tspan,model.r.q(10,:),'b-','LineWidth',2);
%         plot(model.tspan,model.r.q(11,:),'g-','LineWidth',2);
%         plot(model.tspan,model.r.q(12,:),'r-','LineWidth',2);
%         set(gca,'Color','#CCCCCC');
%         xlabel('Time (t) ({\itSeconds})','FontWeight','bold');
%         ylabel('qθ_{1-6} ({\itRadians})','FontWeight','bold');
%         title('Joint Variables: {\itθ}_{7-12}({\itt})','FontSize',12);
%         legend('θ₇','θ₈','θ₉','θ₁₀','θ₁₁','θ₁₂');

%% Animation Pendulum
ANIMATION_PEND = figure(42);
    hold on
    grid on
    for i=1:length(model.tspan)
        cla(ANIMATION_PEND)
        
        title("Pendulum " + model.tspan(i))
        view(135,35);
        axis([-0.15 0.15 -0.1 0.5 0 0.75])
        % ZERO:  Z      X      Y
        plot3([0 1], [0 0], [0 0],'r', 'LineWidth',0.5); % Z
        plot3([0 0], [0 1], [0 0],'g', 'LineWidth',0.5); % X
        plot3([0 0], [0 0], [0 1],'b', 'LineWidth',0.5); % Y
        if i < 2
            legend('+Z','+X','+Y', 'Trajectory','Autoupdate','off');
            xlabel('Z','FontWeight','bold');
            ylabel('X','FontWeight','bold');
            zlabel('Y','FontWeight','bold');
        end
        plot3(model.p.pREF(2,i),... % Y|Z
              model.p.pREF(1,i),... % X
              0,...
              'kx','LineWidth',2,'MarkerSize',5)
        plot3(model.p.x(4,i),...    % Y|Z
              model.p.x(1,i),...    % X
              params.zc,...
              'ro','LineWidth',1,'MarkerSize',5)
        plot3(model.p.x(4,1:i),...  % Y|Z
              model.p.x(1,1:i),...  % X
              params.zc*ones(1,length(model.tspan(1:i))),...
              'r','LineWidth',1)
        drawnow
    end

%% Animation Biped
ANIMATION = figure(69);

for i=1:length(model.tspan)

    cla(ANIMATION)    % Clears Data but not Titles/Labels  
    hold on
    grid on

    txt = " Time: " + num2str(model.tspan(i)) + " sec";
    text(0,0.5,0.5,txt)
    if i > 181
        params.mode     =  1;
    elseif i > 121
        params.mode     = -1;
    elseif i > 61
        params.mode     =  1;
    else 
        params.mode     = -1;
    end

    set(gca,'Color','#CCCCCC');
    [~, HTs]    = k( model.r.q(:,i),i,model,params);

    % ZERO:  Z      X      Y
    plot3([0 1], [0 0], [0 0],'r', 'LineWidth',0.5); % Z
    plot3([0 0], [0 1], [0 0],'g', 'LineWidth',0.5); % X
    plot3([0 0], [0 0], [0 1],'b', 'LineWidth',0.5); % Y
    if i < 2
        legend('+Z','+X','+Y', 'Trajectory','Autoupdate','off');
        title("Stand | Initial Step | Multiple Steps")
        xlabel('Z','FontWeight','bold');
        ylabel('X','FontWeight','bold');
        zlabel('Y','FontWeight','bold');
    end
    % MAIN COMPONENTS
    % Z X Y

    % END EFFECTOR RIGHT!
    r0ER = HTs.A0ER(1:3,4);
    plot3(r0ER(3), r0ER(1), r0ER(2), 'bo', 'LineWidth',0.5,'MarkerSize',10);      % J End E
    % END EFFECTOR LEFT!
    r0EL = HTs.A0EL(1:3,4);
    plot3(r0EL(3), r0EL(1), r0EL(2), 'co', 'LineWidth',0.5,'MarkerSize',10);      % J End E

    % ONE
    r01 = HTs.A01(1:3,4);
    plot3(r01(3), r01(1), r01(2), 'cx', 'LineWidth',2);         % J ONE
    % TWO
    r02 = HTs.A02(1:3,4);
    plot3([r01(3) r02(3)], [r01(1) r02(1)], [r01(2) r02(2)],... % L TWO
    'k', 'LineWidth',2);
    plot3(r02(3), r02(1), r02(2) ,'cx', 'LineWidth',2);         % J TWO
    % THREE
    r03 = HTs.A03(1:3,4);
    plot3([r02(3) r03(3)], [r02(1) r03(1)], [r02(2) r03(2)],... % L THREE
    'k', 'LineWidth',2);
    plot3(r03(3), r03(1), r03(2), 'cx', 'LineWidth',2);         % J THREE
    % FOUR
    r04 = HTs.A04(1:3,4);
    plot3([r03(3) r04(3)], [r03(1) r04(1)], [r03(2) r04(2)],... % L FOUR
    'k', 'LineWidth',2);
    plot3(r04(3), r04(1), r04(2), 'cx', 'LineWidth',2);         % J FOUR
    % FIVE
    r05 = HTs.A05(1:3,4);
    plot3([r04(3) r05(3)], [r04(1) r05(1)], [r04(2) r05(2)],... % L FIVE
    'k', 'LineWidth',2);
    plot3(r05(3), r05(1), r05(2), 'cx', 'LineWidth',2);         % J FIVE
    % SIX 
    r06 = HTs.A06(1:3,4);
    plot3([r05(3) r06(3)], [r05(1) r06(1)], [r05(2) r06(2)],... % L SIX
    'k', 'LineWidth',2);
    plot3(r06(3), r06(1), r06(2), 'cx', 'LineWidth',2);         % J SIX

    % MID WAIST
    r0H = HTs.A0H(1:3,4);
    plot3(r0H(3), r0H(1), r0H(2), 'mx', 'LineWidth',2);         % J MID WST

    % SEVEN 
    r07 = HTs.A07(1:3,4);
    plot3([r06(3) r07(3)], [r06(1) r07(1)], [r06(2) r07(2)],... % L SEVEN
    'k', 'LineWidth',2);
    plot3(r07(3), r07(1), r07(2), 'bx', 'LineWidth',2);         % J SEVEN
    % EIGHT
    r08 = HTs.A08(1:3,4);
    plot3([r07(3) r08(3)], [r07(1) r08(1)], [r07(2) r08(2)],... % L EIGHT
    'k', 'LineWidth',2);
    plot3(r08(3), r08(1), r08(2), 'bx', 'LineWidth',2);         % J EIGHT
    % NINE
    r09 = HTs.A09(1:3,4);
    plot3([r08(3) r09(3)], [r08(1) r09(1)], [r08(2) r09(2)],... % L NINE
    'k', 'LineWidth',2);
    plot3(r09(3), r09(1), r09(2), 'bx', 'LineWidth',2);         % J NINE
    % TEN
    r010= HTs.A010(1:3,4);
    plot3([r09(3) r010(3)], [r09(1) r010(1)], [r09(2) r010(2)],... % L TEN
    'k', 'LineWidth',2);
    plot3(r010(3), r010(1), r010(2), 'bx', 'LineWidth',2);         % J TEN
    % ELEVEN
    r011 = HTs.A011(1:3,4);
    plot3([r010(3) r011(3)], [r010(1) r011(1)], [r010(2) r011(2)],... % L ELEVEN
    'k', 'LineWidth',2);
    plot3(r011(3), r011(1), r011(2), 'bx', 'LineWidth',2);            % J ELEVEN
    % TWELEVE
    r012 = HTs.A0ER(1:3,4);
    plot3([r011(3) r012(3)], [r011(1) r012(1)], [r011(2) r012(2)],... % L TWELEVE
    'k', 'LineWidth',2);
    plot3(r012(3), r012(1), r012(2), 'bx', 'LineWidth',2);            % J TWELEVE
 
    
    plot3(model.r.r0CoMg(3,i),model.r.r0CoMg(1,i),model.r.r0CoMg(2,i),'ro', 'LineWidth',1.5);

    %    [         MIN,          MAX, ...
    axis([          -0.5,        0.5, ...
          (r0H(1))-0.5, (r0H(1)+0.5), ...
                     0,          1]);
    view(135,35);
    % view(90,0); % -> 2D
    IMAGE(i) = getframe(gcf);
    drawnow
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
