clear all
close all
clc 

tic % START TIMING

%% Video/Time Parameters
    params.framerate = 100;                                   % FPS
    model.timestp    = params.framerate^(-1);                % Seconds
    model.tspan      = 0 : model.timestp : 6;               % [ time ]
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
    params.zc        = 0.4564;   % m    - Height of the CoM ^         |
    params.g         = 9.81;     % ms⁻² - Acceleration due to Gravity |
    params.m         = 7.4248;   % kg   - Total Mass of a NuGus       |
 % -------------------------------------------------------------------|
    params.fibula    = 0.4;     % m    - Lower leg
    params.femur     = 0.4;     % m    - Upper Leg
    params.HipWidth  = 0.2;     % m    - Pelvis
    params.ServoSize = 0.05;     % m    - Approximation/Spacing
    params.StepSize  = 0.4;      % m    - 10 cm Step forward
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
                -pi/6;     % θ₂    ->  2D θ₁ Ankle
               2*pi/6;     % θ₃    ->  2D θ₂ Knee
                -pi/6;     % θ₄    ->  2D θ₃ Hip
                    0;     % θ₅
                    0;     % θ₆
                    0;     % θ₇
                    0;     % θ₈
                 pi/6;     % θ₉    ->  2D θ₄ Hip
              -2*pi/6;     % θ₁₀   ->  2D θ₅ Knee
                 pi/6;     % θ₁₁   ->  2D θ₆ Ankle
                    0];    % θ₁₂

    model.r.q(:,1)            = model.r.q0;
   [model.r.xe(:,1), HTs]     = k(model.r.q0, 1, model, params);
    
    model.r.r0Rg(:,1)         = HTs.A0ER(1:3,4);
    model.r.r0Lg(:,1)         = HTs.A0EL(1:3,4);
    model.r.r0Hg(:,1)         = HTs.A0H(1:3,4);
    model.r.r0CoMg(:,1)       = rCoM(model.r.q0,1,model,params);

%% STEP 1  
    % Robot    - Foot Trajectory Generation
        params.mode  = -1;
        stpIndxs     = 1:201;
        [Q,~,~,STEP] = trajectoryGeneration(stpIndxs, 1, model, params);
%%  
    % Pendulum - CoM Trajectory
        % Initial Condidtions
        ZMP0 = STEP(:,1);
        ZMP1 = STEP(:,2);
        model.p.x(:,1) = [model.r.r0CoMg(1,1); 0; 0;  % Position X
                          model.r.r0CoMg(3,1); 0; 0]; % Position Z
        ZMPtim = model.tspan(stpIndxs(end));
        % Load ZMPₓ Reference
        model.p.pREF(:,stpIndxs) = pREF(model.tspan(stpIndxs), ZMP0, ZMP1, ZMPtim);
        % Simulation Loop
        for i=stpIndxs
            [ZMPk, CoMk, model] = LIPM3D(model,i,ZMP0,ZMP1,ZMPtim,params);
            model.r.r0CoMg(:,i) = [CoMk(1); params.zc; CoMk(2)];
        end
    
    for i=stpIndxs(2):stpIndxs(end)
        model.r.xe(:,i)   = [Q(:,i); zeros(3,1)];
        model.r.q(:,i)    = k_Inv(model.r.q(:,i-1), model.r.xe(:,i), i, model, params);
        [~, HTs]          = k(model.r.q(:,i), i-1, model, params);
        model.r.r0Lg(:,i) = HTs.A0EL(1:3,4);
        model.r.r0Rg(:,i) = HTs.A0ER(1:3,4);
        model.r.r0Hg(:,i) = HTs.A0H(1:3,4);
    end

%% STEP 2
    % Robot    - Foot Trajectory Generation
        params.mode = 1;
        stpIndxs = 202:401;
        [Q1,~,~,STEP] = trajectoryGeneration(stpIndxs, 201, model, params);
        Q = [Q Q1];
    % Pendulum - CoM Trajectory
        % Initial Condidtions
        ZMP0 = STEP(:,1);
        ZMP1 = STEP(:,2);
        ZMPtim = model.tspan(stpIndxs(end));
        % Load ZMPₓ Reference
        model.p.pREF(:,stpIndxs) = pREF(model.tspan(stpIndxs), ZMP0, ZMP1, ZMPtim);
        % Simulation Loop
        for i=stpIndxs
            [ZMPk, CoMk, model] = LIPM3D(model,i,ZMP0,ZMP1,ZMPtim,params);
            model.r.r0CoMg(:,i) = [CoMk(1); params.zc; CoMk(2)];
        end
    
    for i=stpIndxs(1):stpIndxs(end)
        model.r.xe(:,i)   = [Q(:,i); zeros(3,1)];
        model.r.q(:,i)    = k_Inv(model.r.q(:,i-1), model.r.xe(:,i), i, model, params);
        [~, HTs]          = k(model.r.q(:,i), i-1, model, params);
        model.r.r0Lg(:,i) = HTs.A0EL(1:3,4);
        model.r.r0Rg(:,i) = HTs.A0ER(1:3,4);
        model.r.r0Hg(:,i) = HTs.A0H(1:3,4);
    end

%% STEP 3
    % Robot    - Foot Trajectory Generation
        params.mode = -1;
        stpIndxs = 402:length(model.tspan);
        [Q2,~,~,STEP] = trajectoryGeneration(stpIndxs, 401, model, params);
        Q = [Q Q2];
    % Pendulum - CoM Trajectory
        % Initial Condidtions
        ZMP0 = STEP(:,1);
        ZMP1 = STEP(:,2);
        ZMPtim = model.tspan(stpIndxs(end));
        % Load ZMPₓ Reference
        model.p.pREF(:,stpIndxs) = pREF(model.tspan(stpIndxs), ZMP0, ZMP1, ZMPtim);
        % Simulation Loop
        for i=stpIndxs
            [ZMPk, CoMk, model] = LIPM3D(model,i,ZMP0,ZMP1,ZMPtim,params);
            model.r.r0CoMg(:,i) = [CoMk(1); params.zc; CoMk(2)];
        end
    
    for i=stpIndxs(1):stpIndxs(end)
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
   
    for i=1:length(model.tspan)
        Pend = subplot(1,2,1);
        hold on
        grid on
        cla(Pend)    % Clears Data but not Titles/Labels  
        
        title("3D LIPM - Preview Control",'FontSize',18);
        view(135,35);
        axis([-0.4,0.2, -1,1, 0,1]);
        % ZERO:  Z      X      Y
        plot3([0 1], [0 0], [0 0],'r', 'LineWidth',2); % Z
        plot3([0 0], [0 1], [0 0],'LineWidth',2,'Color','#379203'); % X
        plot3([0 0], [0 0], [0 1],'b', 'LineWidth',2); % Y
        
        plot3(model.p.y(2,i),... % Y|Z
              model.p.y(1,i),... % X
              0,...
              'kx','LineWidth',3,'MarkerSize',10)
        plot3(model.p.x(4,i),...    % Y|Z
              model.p.x(1,i),...    % X
              params.zc,...
              'mo','LineWidth',3,'MarkerSize',10)
        plot3(model.p.y(2,1:i),...  % Y|Z
              model.p.y(1,1:i),...  % X
              zeros(1,length(model.tspan(1:i))),...
              'k:','LineWidth',1)
        plot3(model.p.x(4,1:i),...  % Y|Z
              model.p.x(1,1:i),...  % X
              params.zc*ones(1,length(model.tspan(1:i))),...
              'm:','LineWidth',1)
        plot3([model.p.y(2,i) model.p.x(4,i)],...  % Y|Z
              [model.p.y(1,i) model.p.x(1,i)],...  % X
              [0 params.zc],...
              'm-','LineWidth',1)

        if i < 2
            legend({'+Z','+X','+Y', ...
                    'ZMP_{out}','CoM'},...
                    'Autoupdate','off','fontsize',16,...
                    'location','west');
            xlabel('{\bfZ} (metres)');
            ylabel('{\bfX} (metres)');
            zlabel('{\bfY} (metres)');
        end

        drawnow
    %   end

% Animation Biped

%for i=1:length(model.tspan)
        Robot = subplot(1,3,3);
        cla(Robot)    % Clears Data but not Titles/Labels  
        hold on
        grid on
    
%         txt = " Time: " + num2str(model.tspan(i)) + " sec";
%         text(0,0.5,0.5,txt)
        
        if i > 401
            params.mode     = -1;
        elseif i > 201
            params.mode     =  1;
        else 
            params.mode     = -1;
        end
    
        set(gca,'Color','#CCCCCC');
        [~, HTs]    = k( model.r.q(:,i),i,model,params);
    
        % ZERO:  Z      X      Y
        plot3([0 1], [0 0], [0 0],'r', 'LineWidth',2); % Z
        plot3([0 0], [0 1], [0 0],'LineWidth',2,'Color','#379203'); % X
        plot3([0 0], [0 0], [0 1],'b', 'LineWidth',2); % Y
        plot3(model.r.r0CoMg(3,i),model.r.r0CoMg(1,i),model.r.r0CoMg(2,i),'mo','LineWidth',2,'MarkerSize',10);
        if i < 2
            legend({'+Z','+X','+Y', 'CoM'},'Autoupdate','off','FontSize',16,'location','east');
            title("3D Model - ZMP Walking",'FontSize',18);
            xlabel('{\bfZ} (metres)');
            ylabel('{\bfX} (metres)');
            zlabel('{\bfY} (metres)');
        end
        % MAIN COMPONENTS
        % Z X Y
    
        % END EFFECTOR RIGHT!
        r0ER = HTs.A0ER(1:3,4);
        plot3(r0ER(3), r0ER(1), r0ER(2), 'bo', 'LineWidth',0.5,'MarkerSize',10);      % J End E
        % END EFFECTOR LEFT!
        r0EL = HTs.A0EL(1:3,4);
        plot3(r0EL(3), r0EL(1), r0EL(2), 'ro', 'LineWidth',0.5,'MarkerSize',10);      % J End E
    
        % ONE
        r01 = HTs.A01(1:3,4);
        plot3(r01(3), r01(1), r01(2), 'rx', 'LineWidth',2,'markersize',10);         % J ONE
        % TWO
        r02 = HTs.A02(1:3,4);
        plot3([r01(3) r02(3)], [r01(1) r02(1)], [r01(2) r02(2)],... % L TWO
        'k', 'LineWidth',2);
        plot3(r02(3), r02(1), r02(2) ,'rx', 'LineWidth',2,'markersize',10);         % J TWO
        % THREE
        r03 = HTs.A03(1:3,4);
        plot3([r02(3) r03(3)], [r02(1) r03(1)], [r02(2) r03(2)],... % L THREE
        'k', 'LineWidth',2);
        plot3(r03(3), r03(1), r03(2), 'rx', 'LineWidth',2,'markersize',10);         % J THREE
        % FOUR
        r04 = HTs.A04(1:3,4);
        plot3([r03(3) r04(3)], [r03(1) r04(1)], [r03(2) r04(2)],... % L FOUR
        'k', 'LineWidth',2);
        plot3(r04(3), r04(1), r04(2), 'rx', 'LineWidth',2,'markersize',10);         % J FOUR
        % FIVE
        r05 = HTs.A05(1:3,4);
        plot3([r04(3) r05(3)], [r04(1) r05(1)], [r04(2) r05(2)],... % L FIVE
        'k', 'LineWidth',2);
        plot3(r05(3), r05(1), r05(2), 'rx', 'LineWidth',2,'markersize',10);         % J FIVE
        % SIX 
        r06 = HTs.A06(1:3,4);
        plot3([r05(3) r06(3)], [r05(1) r06(1)], [r05(2) r06(2)],... % L SIX
        'k', 'LineWidth',2);
        plot3(r06(3), r06(1), r06(2), 'rx', 'LineWidth',2,'markersize',10);         % J SIX

        % SEVEN 
        r07 = HTs.A07(1:3,4);
        plot3([r06(3) r07(3)], [r06(1) r07(1)], [r06(2) r07(2)],... % L SEVEN
        'k', 'LineWidth',2);
        plot3(r07(3), r07(1), r07(2), 'bx', 'LineWidth',2,'markersize',10);         % J SEVEN
        % EIGHT
        r08 = HTs.A08(1:3,4);
        plot3([r07(3) r08(3)], [r07(1) r08(1)], [r07(2) r08(2)],... % L EIGHT
        'k', 'LineWidth',2);
        plot3(r08(3), r08(1), r08(2), 'bx', 'LineWidth',2,'markersize',10);         % J EIGHT
        % NINE
        r09 = HTs.A09(1:3,4);
        plot3([r08(3) r09(3)], [r08(1) r09(1)], [r08(2) r09(2)],... % L NINE
        'k', 'LineWidth',2);
        plot3(r09(3), r09(1), r09(2), 'bx', 'LineWidth',2,'markersize',10);         % J NINE
        % TEN
        r010= HTs.A010(1:3,4);
        plot3([r09(3) r010(3)], [r09(1) r010(1)], [r09(2) r010(2)],... % L TEN
        'k', 'LineWidth',2);
        plot3(r010(3), r010(1), r010(2), 'bx', 'LineWidth',2,'markersize',10);         % J TEN
        % ELEVEN
        r011 = HTs.A011(1:3,4);
        plot3([r010(3) r011(3)], [r010(1) r011(1)], [r010(2) r011(2)],... % L ELEVEN
        'k', 'LineWidth',2);
        plot3(r011(3), r011(1), r011(2), 'bx', 'LineWidth',2,'markersize',10);            % J ELEVEN
        % TWELEVE
        r012 = HTs.A0ER(1:3,4);
        plot3([r011(3) r012(3)], [r011(1) r012(1)], [r011(2) r012(2)],... % L TWELEVE
        'k', 'LineWidth',2);
        plot3(r012(3), r012(1), r012(2), 'bx', 'LineWidth',2,'markersize',10);            % J TWELEVE
     
        
        plot3(model.p.pREF(2,i),... % Y|Z
                  model.p.pREF(1,i),... % X
                  0,...
                  'kx','LineWidth',2,'MarkerSize',5)
        plot3([model.p.pREF(2,i), model.p.x(4,i)],... % Y|Z
              [model.p.pREF(1,i), model.p.x(1,i)],... % X
              [0, params.zc],...
              'm:','LineWidth',2)
        plot3(model.p.x(4,i),...    % Y|Z
              model.p.x(1,i),...    % X
              params.zc,...
              'mx','LineWidth',1,'MarkerSize',5)
        plot3(model.p.x(4,1:i),...  % Y|Z
              model.p.x(1,1:i),...  % X
              params.zc*ones(1,length(model.tspan(1:i))),...
              'm','LineWidth',1)
    
    
        %    [         MIN,          MAX, ...
        axis([          -0.5,        0.3, ...
              model.r.r0CoMg(3,i)-1, model.r.r0CoMg(1,i)+1, ...
                         0,          1]);
        view(150,20);
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
