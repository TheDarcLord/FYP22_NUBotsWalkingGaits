clear all
close all
clc 

tic % START TIMING

%% Setup
params.framerate    = 10;
model.tspan         = 0:(1 / params.framerate):30;

model.q             = zeros(12,length(model.tspan)); % q     [θ₁θ₂θ₃ ...]ᵀ
model.xe            = zeros(6,length(model.tspan));  % xe    [XYZϕθΨ]ᵀ
model.r0Lg          = zeros(3,length(model.tspan));  % A0EL  [XYZ]ᵀ
model.r0Rg          = zeros(3,length(model.tspan));  % A0ER  [XYZ]ᵀ
model.r0Hg          = zeros(3,length(model.tspan));  % A0H   [XYZ]ᵀ
model.r0CoMg        = zeros(3,length(model.tspan));  % r0CoMg  [XYZ]ᵀ

% Physical Parameters
params.fibula       = 0.4;
params.femur        = 0.4;
params.HipWidth     = 0.2;
params.ServoSize    = 0.05;

params.StepSize     = 0.4;
params.r0Lg         = [0.2; 0; -0.1];  % Right Position from 0rigin in Global
params.r0Hg         = zeros(3,1);  % Waist Position from 0rigin in Global
params.r0Rg         = zeros(3,1);  % Left  Position from 0rigin in Global
params.r0CoMg       = zeros(3,1);  % CoM   Position from 0rigin in Global
params.mode         = -1;          % LEFT  FIXED - FKM T16
%                      0;          % BOTH  FIXED - FKM T1H T6H
%                      1;          % RIGHT FIXED - FKM T61
% Masses
params.mass.femur   = 1;    % Thigh Bone
params.mass.fibula  = 1;    % Paired with `tibia`
params.mass.joint   = 0.5;  % Knee Bone / Joints
params.mass.pelvis  = 1.5;  % Waist

%% Initial Position & Orientation

model.q0 = [    0;     % θ₁    
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

model.q(:,1)            = model.q0;
[model.xe(:,1), HTs]    = k(model.q0, params);

model.r0Rg(:,1)         = HTs.A0ER(1:3,4);
    params.r0Rg         = model.r0Rg(:,1);
model.r0Lg(:,1)         = HTs.A0EL(1:3,4);
    params.r0Lg         = model.r0Lg(:,1);
model.r0Hg(:,1)         = HTs.A0H(1:3,4);
    params.r0Hg         = model.r0Hg(:,1);
model.r0CoMg(:,1)       = rCoM(model.q(:,1),params);
    params.r0CoMg       = model.r0CoMg(:,1);

%% Initial Figure
i = 1;
InitialFigure = figure(1);
    cla    % Clears Data but not Titles/Labels  
    hold on
    grid on
    set(gca,'Color','#CCCCCC');
    view(145,20);
    
    params.r0Lg     = model.r0Lg(:,i);
    params.r0Rg     = model.r0Rg(:,i);
    params.r0Hg     = model.r0Hg(:,i);
    params.r0CoMg   = model.r0CoMg(:,i);

    set(gca,'Color','#CCCCCC');
    [~, HTs]    = k(model.q(:,i), params);

    % ZERO:  Z      X      Y
    plot3([0 1], [0 0], [0 0],'r', 'LineWidth',3); % Z
    plot3([0 0], [0 1], [0 0], 'LineWidth',3,'Color','#379203'); % X
    plot3([0 0], [0 0], [0 1],'b', 'LineWidth',3); % Y
    quiver3(0,0,0,model.r0Lg(3,1),model.r0Lg(1,1),0,...
        'LineWidth',2,'Color','k','ShowArrowHead','on',...
        'LineStyle',':','AutoScale','off')
    title("3D Model",'FontSize',18);
    xlabel('{\bfZ} (metres)');
    ylabel('{\bfX} (metres)');
    zlabel('{\bfY} (metres)');
    % MAIN COMPONENTS

    % END EFFECTOR RIGHT!
    r0ER = HTs.A0ER(1:3,4);
    plot3(r0ER(3), r0ER(1), r0ER(2), 'bo', 'LineWidth',0.5,'MarkerSize',10);      % J End E
    % END EFFECTOR LEFT!
    r0EL = HTs.A0EL(1:3,4);
    plot3(r0EL(3), r0EL(1), r0EL(2), 'ro', 'LineWidth',0.5,'MarkerSize',10);      % J End E

    % ONE
    r01 = HTs.A01(1:3,4);
    plot3(r01(3), r01(1), r01(2), 'rx', 'LineWidth',3,'MarkerSize',10);         % J ONE
    % TWO
    r02 = HTs.A02(1:3,4);
    plot3([r01(3) r02(3)], [r01(1) r02(1)], [r01(2) r02(2)],... % L TWO
    'k', 'LineWidth',2);
    plot3(r02(3), r02(1), r02(2) ,'rx', 'LineWidth',3,'MarkerSize',10);         % J TWO
    % THREE
    r03 = HTs.A03(1:3,4);
    plot3([r02(3) r03(3)], [r02(1) r03(1)], [r02(2) r03(2)],... % L THREE
    'k', 'LineWidth',2);
    plot3(r03(3), r03(1), r03(2), 'rx', 'LineWidth',3,'MarkerSize',10);         % J THREE
    % FOUR
    r04 = HTs.A04(1:3,4);
    plot3([r03(3) r04(3)], [r03(1) r04(1)], [r03(2) r04(2)],... % L FOUR
    'k', 'LineWidth',2);
    plot3(r04(3), r04(1), r04(2), 'rx', 'LineWidth',3,'MarkerSize',10);         % J FOUR
    % FIVE
    r05 = HTs.A05(1:3,4);
    plot3([r04(3) r05(3)], [r04(1) r05(1)], [r04(2) r05(2)],... % L FIVE
    'k', 'LineWidth',2);
    plot3(r05(3), r05(1), r05(2), 'rx', 'LineWidth',3,'MarkerSize',10);         % J FIVE
    % SIX 
    r06 = HTs.A06(1:3,4);
    plot3([r05(3) r06(3)], [r05(1) r06(1)], [r05(2) r06(2)],... % L SIX
    'k', 'LineWidth',2);
    plot3(r06(3), r06(1), r06(2), 'rx', 'LineWidth',3,'MarkerSize',10);         % J SIX

    % MID WAIST
    r0H = HTs.A0H(1:3,4);
    plot3(r0H(3), r0H(1), r0H(2), 'mx', 'LineWidth',3,'MarkerSize',10);         % J MID WST

    % SEVEN 
    r07 = HTs.A07(1:3,4);
    plot3([r06(3) r07(3)], [r06(1) r07(1)], [r06(2) r07(2)],... % L SEVEN
    'k', 'LineWidth',2);
    plot3(r07(3), r07(1), r07(2), 'bx', 'LineWidth',3,'MarkerSize',10);         % J SEVEN
    % EIGHT
    r08 = HTs.A08(1:3,4);
    plot3([r07(3) r08(3)], [r07(1) r08(1)], [r07(2) r08(2)],... % L EIGHT
    'k', 'LineWidth',2);
    plot3(r08(3), r08(1), r08(2), 'bx', 'LineWidth',3,'MarkerSize',10);         % J EIGHT
    % NINE
    r09 = HTs.A09(1:3,4);
    plot3([r08(3) r09(3)], [r08(1) r09(1)], [r08(2) r09(2)],... % L NINE
    'k', 'LineWidth',2);
    plot3(r09(3), r09(1), r09(2), 'bx', 'LineWidth',3,'MarkerSize',10);         % J NINE
    % TEN
    r010= HTs.A010(1:3,4);
    plot3([r09(3) r010(3)], [r09(1) r010(1)], [r09(2) r010(2)],... % L TEN
    'k', 'LineWidth',2);
    plot3(r010(3), r010(1), r010(2), 'bx', 'LineWidth',3,'MarkerSize',10);         % J TEN
    % ELEVEN
    r011 = HTs.A011(1:3,4);
    plot3([r010(3) r011(3)], [r010(1) r011(1)], [r010(2) r011(2)],... % L ELEVEN
    'k', 'LineWidth',2);
    plot3(r011(3), r011(1), r011(2), 'bx', 'LineWidth',3,'MarkerSize',10);            % J ELEVEN
    % TWELEVE
    r012 = HTs.A0ER(1:3,4);
    plot3([r011(3) r012(3)], [r011(1) r012(1)], [r011(2) r012(2)],... % L TWELEVE
    'k', 'LineWidth',2);
    plot3(r012(3), r012(1), r012(2), 'bx', 'LineWidth',3,'MarkerSize',10);            % J TWELEVE

    legend({'+Z_0','+X_0','+Y_0','{r}^0_{E_{Left}} - \it{Link 1}',...
            '','','Joints 1 - 6','','','','','','','','','','',...
            'Mid Waist','','Joints 7 - 12'},...
            'FontSize',16,Location='west');
    
    axis([-0.4,0.2, -1,1, 0,1]);


%% LOOP
% Initial Conditions
model.q(:,1)            = model.q0;
[model.xe(:,1), HTs]    = k(model.q0, params);

model.r0Rg(:,1)         = HTs.A0ER(1:3,4);
    params.r0Rg         = model.r0Rg(:,1);
model.r0Lg(:,1)         = HTs.A0EL(1:3,4);
    params.r0Lg         = model.r0Lg(:,1);
model.r0Hg(:,1)         = HTs.A0H(1:3,4);
    params.r0Hg         = model.r0Hg(:,1);
model.r0CoMg(:,1)       = rCoM(model.q(:,1),params);
    params.r0CoMg       = model.r0CoMg(:,1);

params.mode = -1;
% Trajectory Generation
[Q1,~,~] = trajectoryGeneration(model, 1:61,params);

for i=2:61
    model.xe(:,i)   = [Q1(:,i); zeros(3,1)];
    model.q(:,i)    = k_Inv(model.q(:,i-1), model.xe(:,i), params);
    [~, HTs]        = k(model.q(:,i), params);
    model.r0Lg(:,i) = HTs.A0EL(1:3,4);
     params.r0Lg    = model.r0Lg(:,i);
    model.r0Rg(:,i) = HTs.A0ER(1:3,4);
     params.r0Rg    = model.r0Rg(:,i);
    model.r0Hg(:,i) = HTs.A0H(1:3,4);
     params.r0Hg    = model.r0Hg(:,i);
    model.r0CoMg(:,i) = rCoM(model.q(:,i),params);
     params.r0CoMg  = model.r0CoMg(:,i);
end


params.mode = 0;
% Trajectory Generation
[Q2,~,~] = trajectoryGeneration(model, 62:121,params);

for i=62:121
    model.xe(:,i)   = [Q2(:,i-61); zeros(3,1)];
    model.q(:,i)    = k_Inv(model.q(:,i-1), model.xe(:,i), params);
    [~, HTs]        = k(model.q(:,i), params);
    model.r0Lg(:,i) = HTs.A0EL(1:3,4);
     params.r0Lg    = model.r0Lg(:,i);
    model.r0Rg(:,i) = HTs.A0ER(1:3,4);
     params.r0Rg    = model.r0Rg(:,i);
    model.r0Hg(:,i) = HTs.A0H(1:3,4);
     params.r0Hg    = model.r0Hg(:,i);
    model.r0CoMg(:,i) = rCoM(model.q(:,i),params);
     params.r0CoMg  = model.r0CoMg(:,i);
end

params.mode = 1;
% Trajectory Generation
[Q3,~,~] = trajectoryGeneration(model, 122:181,params); 

for i=122:181
    model.xe(:,i)   = [Q3(:,i-121); zeros(3,1)];
    model.q(:,i)    = k_Inv(model.q(:,i-1), model.xe(:,i), params);
    [~, HTs]        = k(model.q(:,i), params);
    model.r0Lg(:,i) = HTs.A0EL(1:3,4);
     params.r0Lg    = model.r0Lg(:,i);
    model.r0Rg(:,i) = HTs.A0ER(1:3,4);
     params.r0Rg    = model.r0Rg(:,i);
    model.r0Hg(:,i) = HTs.A0H(1:3,4);
     params.r0Hg    = model.r0Hg(:,i);
    model.r0CoMg(:,i) = rCoM(model.q(:,i),params);
     params.r0CoMg  = model.r0CoMg(:,i);
end

params.mode = 0;
% Trajectory Generation
[Q4,~,~] = trajectoryGeneration(model, 182:241,params);

for i=182:241
    model.xe(:,i)   = [Q4(:,i-181); zeros(3,1)];
    model.q(:,i)    = k_Inv(model.q(:,i-1), model.xe(:,i), params);
    [~, HTs]        = k(model.q(:,i), params);
    model.r0Lg(:,i) = HTs.A0EL(1:3,4);
     params.r0Lg    = model.r0Lg(:,i);
    model.r0Rg(:,i) = HTs.A0ER(1:3,4);
     params.r0Rg    = model.r0Rg(:,i);
    model.r0Hg(:,i) = HTs.A0H(1:3,4);
     params.r0Hg    = model.r0Hg(:,i);
    model.r0CoMg(:,i) = rCoM(model.q(:,i),params);
     params.r0CoMg  = model.r0CoMg(:,i);
end

params.mode = -1;
% Trajectory Generation
[Q5,~,~] = trajectoryGeneration(model, 1:61,params);

for i=242:301
    model.xe(:,i)   = [Q5(:,i-241); zeros(3,1)];
    model.q(:,i)    = k_Inv(model.q(:,i-1), model.xe(:,i), params);
    [~, HTs]        = k(model.q(:,i), params);
    model.r0Lg(:,i) = HTs.A0EL(1:3,4);
     params.r0Lg    = model.r0Lg(:,i);
    model.r0Rg(:,i) = HTs.A0ER(1:3,4);
     params.r0Rg    = model.r0Rg(:,i);
    model.r0Hg(:,i) = HTs.A0H(1:3,4);
     params.r0Hg    = model.r0Hg(:,i);
    model.r0CoMg(:,i) = rCoM(model.q(:,i),params);
     params.r0CoMg  = model.r0CoMg(:,i);
end

toc % FINISH TIMING

%% Figures 
figure('Name','Joint Variables, q(t)')
    subplot(1,2,1)
        hold on
        plot(model.tspan,model.q(1,:),'r-','LineWidth',2);
        plot(model.tspan,model.q(2,:),'g-','LineWidth',2);
        plot(model.tspan,model.q(3,:),'b-','LineWidth',2);
        plot(model.tspan,model.q(4,:),'c-','LineWidth',2);
        plot(model.tspan,model.q(5,:),'y-','LineWidth',2);
        plot(model.tspan,model.q(6,:),'m-','LineWidth',2);
        set(gca,'Color','#CCCCCC');
        xlabel('Time (t) ({\itSeconds})','FontWeight','bold');
        ylabel('qθ_{1-6} ({\itRadians})','FontWeight','bold');
        title('Joint Variables: {\itθ}_{1-6}({\itt})','FontSize',12);
        legend('θ₁','θ₂','θ₃', 'θ₄','θ₅','θ₆');
    subplot(1,2,2)
        hold on
        plot(model.tspan,model.q(7,:),'m-','LineWidth',2);
        plot(model.tspan,model.q(8,:),'y-','LineWidth',2);
        plot(model.tspan,model.q(9,:),'c-','LineWidth',2);
        plot(model.tspan,model.q(10,:),'b-','LineWidth',2);
        plot(model.tspan,model.q(11,:),'g-','LineWidth',2);
        plot(model.tspan,model.q(12,:),'r-','LineWidth',2);
        set(gca,'Color','#CCCCCC');
        xlabel('Time (t) ({\itSeconds})','FontWeight','bold');
        ylabel('qθ_{1-6} ({\itRadians})','FontWeight','bold');
        title('Joint Variables: {\itθ}_{7-12}({\itt})','FontSize',12);
        legend('θ₇','θ₈','θ₉','θ₁₀','θ₁₁','θ₁₂');

figure('Name','Foot,Waist,CoM Movement')
    hold on
    grid on
    title('Foot & Waist & CoM Trajectories','FontSize',12);
    set(gca,'Color','#CCCCCC');
    plot3([0 1], [0 0], [0 0],'r', 'LineWidth',0.5); % Z
    plot3([0 0], [0 1], [0 0],'g', 'LineWidth',0.5); % X
    plot3([0 0], [0 0], [0 1],'b', 'LineWidth',0.5); % Y
    plot3(model.r0Lg(3,:),model.r0Lg(1,:),model.r0Lg(2,:),...
        'c-','LineWidth',2);
    plot3(model.r0Rg(3,:),model.r0Rg(1,:),model.r0Rg(2,:),...
        'g-','LineWidth',2);
    plot3(model.r0Hg(3,:),model.r0Hg(1,:),model.r0Hg(2,:),...
        'm-','LineWidth',2);
    plot3(model.r0CoMg(3,:),model.r0CoMg(1,:),model.r0CoMg(2,:),...
        'r-','LineWidth',1);

    legend('+Z','+X','+Y','Left','Right','Waist', 'CoM');
    axis([-(params.HipWidth+0.2) 0.2 min(model.r0Hg(1,:))-0.1 max(model.r0Hg(1,:))+0.2 0 2]);
    view(135,35);

figure('Name','Animation')
% Preallocate IMAGE
[IMAGE(1:length(model.tspan)).cdata]    = deal([]); 
[IMAGE(1:length(model.tspan)).colormap] = deal([]); 
for i=1:length(model.tspan)

    cla    % Clears Data but not Titles/Labels  
    hold on
    grid on
    
    txt = " Time: " + num2str(model.tspan(i)) + " sec";
    text(0,2,2,txt)
    if i > 241
        params.mode     = -1;
    elseif i > 181    
        params.mode     =  0;
    elseif i > 121
        params.mode     =  1;
    elseif i > 61
        params.mode     =  0;
    else 
        params.mode     = -1;
    end
    
    params.r0Lg     = model.r0Lg(:,i);
    params.r0Rg     = model.r0Rg(:,i);
    params.r0Hg     = model.r0Hg(:,i);
    params.r0CoMg   = model.r0CoMg(:,i);

    set(gca,'Color','#CCCCCC');
    [~, HTs]    = k(model.q(:,i), params);

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

    % END EFFECTOR RIGHT!
    r0ER = HTs.A0ER(1:3,4);
    plot3(r0ER(3), r0ER(1), r0ER(2), 'bo', 'LineWidth',0.5,'MarkerSize',10);      % J End E
    % END EFFECTOR LEFT!
    r0EL = HTs.A0EL(1:3,4);
    plot3(r0EL(3), r0EL(1), r0EL(2), 'co', 'LineWidth',0.5,'MarkerSize',10);      % J End E

    % ONE
    r01 = HTs.A01(1:3,4);
    plot3(r01(3), r01(1), r01(2), 'rx', 'LineWidth',3,'MarkerSize',10);         % J ONE
    % TWO
    r02 = HTs.A02(1:3,4);
    plot3([r01(3) r02(3)], [r01(1) r02(1)], [r01(2) r02(2)],... % L TWO
    'k', 'LineWidth',2);
    plot3(r02(3), r02(1), r02(2) ,'rx', 'LineWidth',3,'MarkerSize',10);         % J TWO
    % THREE
    r03 = HTs.A03(1:3,4);
    plot3([r02(3) r03(3)], [r02(1) r03(1)], [r02(2) r03(2)],... % L THREE
    'k', 'LineWidth',2);
    plot3(r03(3), r03(1), r03(2), 'rx', 'LineWidth',3,'MarkerSize',10);         % J THREE
    % FOUR
    r04 = HTs.A04(1:3,4);
    plot3([r03(3) r04(3)], [r03(1) r04(1)], [r03(2) r04(2)],... % L FOUR
    'k', 'LineWidth',2);
    plot3(r04(3), r04(1), r04(2), 'rx', 'LineWidth',3,'MarkerSize',10);         % J FOUR
    % FIVE
    r05 = HTs.A05(1:3,4);
    plot3([r04(3) r05(3)], [r04(1) r05(1)], [r04(2) r05(2)],... % L FIVE
    'k', 'LineWidth',2);
    plot3(r05(3), r05(1), r05(2), 'rx', 'LineWidth',3,'MarkerSize',10);         % J FIVE
    % SIX 
    r06 = HTs.A06(1:3,4);
    plot3([r05(3) r06(3)], [r05(1) r06(1)], [r05(2) r06(2)],... % L SIX
    'k', 'LineWidth',2);
    plot3(r06(3), r06(1), r06(2), 'rx', 'LineWidth',3,'MarkerSize',10);         % J SIX

    % MID WAIST
    r0H = HTs.A0H(1:3,4);
    plot3(r0H(3), r0H(1), r0H(2), 'mx', 'LineWidth',3,'MarkerSize',10);         % J MID WST

    % SEVEN 
    r07 = HTs.A07(1:3,4);
    plot3([r06(3) r07(3)], [r06(1) r07(1)], [r06(2) r07(2)],... % L SEVEN
    'k', 'LineWidth',2);
    plot3(r07(3), r07(1), r07(2), 'bx', 'LineWidth',3,'MarkerSize',10);         % J SEVEN
    % EIGHT
    r08 = HTs.A08(1:3,4);
    plot3([r07(3) r08(3)], [r07(1) r08(1)], [r07(2) r08(2)],... % L EIGHT
    'k', 'LineWidth',2);
    plot3(r08(3), r08(1), r08(2), 'bx', 'LineWidth',3,'MarkerSize',10);         % J EIGHT
    % NINE
    r09 = HTs.A09(1:3,4);
    plot3([r08(3) r09(3)], [r08(1) r09(1)], [r08(2) r09(2)],... % L NINE
    'k', 'LineWidth',2);
    plot3(r09(3), r09(1), r09(2), 'bx', 'LineWidth',3,'MarkerSize',10);         % J NINE
    % TEN
    r010= HTs.A010(1:3,4);
    plot3([r09(3) r010(3)], [r09(1) r010(1)], [r09(2) r010(2)],... % L TEN
    'k', 'LineWidth',2);
    plot3(r010(3), r010(1), r010(2), 'bx', 'LineWidth',3,'MarkerSize',10);         % J TEN
    % ELEVEN
    r011 = HTs.A011(1:3,4);
    plot3([r010(3) r011(3)], [r010(1) r011(1)], [r010(2) r011(2)],... % L ELEVEN
    'k', 'LineWidth',2);
    plot3(r011(3), r011(1), r011(2), 'bx', 'LineWidth',3,'MarkerSize',10);            % J ELEVEN
    % TWELEVE
    r012 = HTs.A0ER(1:3,4);
    plot3([r011(3) r012(3)], [r011(1) r012(1)], [r011(2) r012(2)],... % L TWELEVE
    'k', 'LineWidth',2);
    plot3(r012(3), r012(1), r012(2), 'bx', 'LineWidth',3,'MarkerSize',10);            % J TWELEVE

    plot3(params.r0CoMg(3),params.r0CoMg(1),params.r0CoMg(2), 'ro', 'LineWidth',1.5);

    %    [         MIN,          MAX, ...
    axis([          -1,            1, ...
          (r0H(1))-1.1, (r0H(1)+1.1), ...
                     0,            2]);
    view(135,35);
    % view(90,0); % -> 2D
    IMAGE(i) = getframe(gcf);
    drawnow
end

%% VIDEO
videoWriterObj           = VideoWriter('3D_Step.mp4','MPEG-4');
videoWriterObj.FrameRate = params.framerate*2; % 15sec video
open(videoWriterObj);                        
for i=1:length(IMAGE)
    frame = IMAGE(i);   % Convert from an Image to a Frame
    writeVideo(videoWriterObj, frame);
end
close(videoWriterObj);
