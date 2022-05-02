clear all
close all
clc 

tic % START TIMING

%% Setup
params.framerate    = 10;
model.tspan         = 0:(1 / params.framerate):24;

model.q             = zeros(12,length(model.tspan)); % q    [θ₁θ₂θ₃ ...]ᵀ
model.xe            = zeros(7,length(model.tspan)); % xe    [XYZϕθΨ]ᵀ
model.r01g          = zeros(3,length(model.tspan)); % A01   [XYZ]ᵀ
model.r012g         = zeros(3,length(model.tspan)); % A0_12 [XYZ]ᵀ
model.r0Hg          = zeros(3,length(model.tspan)); % A0H   [XYZ]ᵀ
%model.rCoM          = zeros(3,length(model.tspan)); % rCoM  [XYZ]ᵀ

% Physical Parameters
params.fibula       = 0.5;
params.femur        = 0.5;
params.HipWidth     = 0.25;
params.StepSize     = 0.4;
params.r0Lg         = zeros(3,1);  % Right Position from 0rigin in Global
params.r0Hg         = zeros(3,1);  % Waist Position from 0rigin in Global
params.r0Rg         = zeros(3,1);  % Left  Position from 0rigin in Global
%params.r0CoMg       = zeros(3,1);  % CoM   Position from 0rigin in Global
params.mode         = -1;          % LEFT  FIXED - FKM T16
%                      0;          % BOTH  FIXED - FKM T1H T6H
%                      1;          % RIGHT FIXED - FKM T61
% Masses
params.mass.femur   = 1;    % Thigh Bone
params.mass.fibula  = 1;    % Paired with `tibia`
params.mass.joint   = 0.5;  % Knee Bone / Joints
params.mass.pelvis  = 1.5;  % Waist

%% Initial Position & Orientation

model.q0 = [-pi/12;     % θ₁    ->  2D θ₁ Ankle
                 0;     % θ₂
           2*pi/12;     % θ₃    ->  2D θ₂ Knee
            -pi/12;     % θ₄    ->  2D θ₃ Hip
                 0;     % θ₅
                 0;     % θ₆
                 0;     % θ₇
                 0;     % θ₈
             pi/12;     % θ₉    ->  2D θ₄ Hip
          -2*pi/12;     % θ₁₀   ->  2D θ₅ Knee
                 0;     % θ₁₁
             pi/12];    % θ₁₂   ->  2D θ₆ Ankle

%% LOOP
% Initial Conditions
model.q(:,1)                = model.q0;
[model.xe(:,1), ~, HTs]     = k(model.q0, params);


model.r012g(:,1)            = HTs.A012(1:3,4);
model.r01g(:,1)             = HTs.A01(1:3,4);
model.r0Hg(:,1)             = HTs.A0H(1:3,4);
params.waistHeight          = model.r0Hg(2,1);

params.r0Lg                 = model.r01g(:,1);
params.r0Rg                 = model.r012g(:,1);
params.r0Hg                 = model.r0Hg(:,1);
%model.rCoM(:,1)             = rCoM(HTs,params);
%params.r0CoMg               = model.rCoM(:,1);

[Q1,~,~] = trajectoryGeneration(model, 1:61,params); % Trajectory Generation

for i=2:61
    model.xe(:,i)   = [Q1(:,i); zeros(3,1); params.waistHeight];
    model.q(:,i)    = k_Inv(model.q(:,i-1), model.xe(:,i), params);
    [~, ~, HTs]     = k(model.q(:,i), params);
    model.r01g(:,i) = HTs.A01(1:3,4);
    model.r012g(:,i) = HTs.A012(1:3,4);
    model.r0Hg(:,i) = HTs.A0H(1:3,4);
%    model.rCoM(:,i) = rCoM(HTs,params);
end

params.r0Lg                 = model.r01g(:,61);
params.r0Rg                 = model.r012g(:,61);
params.r0Hg                 = model.r0Hg(:,61);
%params.r0CoMg               = model.rCoM(:,61);
params.mode = 1;
[Q2,~,~] = trajectoryGeneration(model, 62:121,params); % Trajectory Generation

for i=62:121
    model.xe(:,i)   = [Q2(:,i-61); zeros(3,1); params.waistHeight];
    model.q(:,i)    = k_Inv(model.q(:,i-1), model.xe(:,i), params);
    [~, ~, HTs]     = k(model.q(:,i), params);
    model.r01g(:,i) = HTs.A01(1:3,4);
    model.r012g(:,i) = HTs.A012(1:3,4);
    model.r0Hg(:,i) = HTs.A0H(1:3,4);
%    model.rCoM(:,i) = rCoM(HTs,params);
end

params.r0Lg                 = model.r01g(:,121);
params.r0Rg                 = model.r012g(:,121);
params.r0Hg                 = model.r0Hg(:,121);
%params.r0CoMg               = model.rCoM(:,121);
params.mode = -1;
[Q3,~,~] = trajectoryGeneration(model, 122:181,params); % Trajectory Generation

for i=122:181
    model.xe(:,i)   = [Q3(:,i-121); zeros(3,1); params.waistHeight];
    model.q(:,i)    = k_Inv(model.q(:,i-1), model.xe(:,i), params);
    [~, ~, HTs]     = k(model.q(:,i), params);
    model.r01g(:,i) = HTs.A01(1:3,4);
    model.r012g(:,i) = HTs.A012(1:3,4);
    model.r0Hg(:,i) = HTs.A0H(1:3,4);
%    model.rCoM(:,i) = rCoM(HTs,params);
end

params.r0Lg                 = model.r01g(:,181);
params.r0Rg                 = model.r012g(:,181);
params.r0Hg                 = model.r0Hg(:,181);
%params.r0CoMg               = model.rCoM(:,181);
params.mode = 1;
[Q4,~,~] = trajectoryGeneration(model, 182:241,params); % Trajectory Generation

for i=182:241
    model.xe(:,i)   = [Q4(:,i-181); zeros(3,1); params.waistHeight];
    model.q(:,i)    = k_Inv(model.q(:,i-1), model.xe(:,i), params);
    [~, ~, HTs]     = k(model.q(:,i), params);
    model.r01g(:,i) = HTs.A01(1:3,4);
    model.r012g(:,i) = HTs.A012(1:3,4);
    model.r0Hg(:,i) = HTs.A0H(1:3,4);
%    model.rCoM(:,i) = rCoM(HTs,params);
end

toc % FINISH TIMING

%% Figures 
% figure('Name','Joint Variables, q(t)')
%     hold on
%     plot(model.tspan,model.q(1,:),'r-','LineWidth',2);
%     plot(model.tspan,model.q(2,:),'g-','LineWidth',2);
%     plot(model.tspan,model.q(3,:),'b-','LineWidth',2);
%     plot(model.tspan,model.q(4,:),'c-','LineWidth',2);
%     plot(model.tspan,model.q(5,:),'y-','LineWidth',2);
%     plot(model.tspan,model.q(6,:),'m-','LineWidth',2);
%     set(gca,'Color','#CCCCCC');
%     xlabel('Time (t) ({\itSeconds})','FontWeight','bold');
%     ylabel('qθ_{1-6} ({\itRadians})','FontWeight','bold');
%     title('All Joint Variables: {\itθ}_{1-6}({\itt})','FontSize',12);
%     legend('θ₁','θ₂','θ₃', 'θ₄','θ₅','θ₆');

% figure('Name','Foot,Waist,CoM Movement')
%     hold on
%     grid on
%     title('Foot & Waist & CoM Trajectories','FontSize',12);
%     set(gca,'Color','#CCCCCC');
%     plot3([0 1], [0 0], [0 0],'r', 'LineWidth',0.5); % Z
%     plot3([0 0], [0 1], [0 0],'g', 'LineWidth',0.5); % X
%     plot3([0 0], [0 0], [0 1],'b', 'LineWidth',0.5); % Y
%     plot3(model.r01g(3,:),model.r01g(1,:),model.r01g(2,:),...
%         'c-','LineWidth',2);
%     plot3(model.r06g(3,:),model.r06g(1,:),model.r06g(2,:),...
%         'g-','LineWidth',2);
%     plot3(model.r0Hg(3,:),model.r0Hg(1,:),model.r0Hg(2,:),...
%         'm-','LineWidth',2);
%     plot3(model.rCoM(3,:),model.rCoM(1,:),model.rCoM(2,:),...
%         'r-','LineWidth',1);
% 
%     legend('+Z','+X','+Y','Left','Right','Waist', 'CoM');
%     axis([-(params.HipWidth+0.2) 0.2 min(model.r0Hg(1,:))-0.1 max(model.r0Hg(1,:))+0.2 0 1]);
%     view(135,35);

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
    if i > 181
        params.r0Lg     = model.r01g(:,181);
        params.r0Rg     = model.r012g(:,181);
        params.r0Hg     = model.r0Hg(:,181);
%        params.r0CoMg   = model.rCoM(:,181);
        params.mode     =  1;
    elseif i > 121
        params.r0Lg     = model.r01g(:,121);
        params.r0Rg     = model.r012g(:,121);
        params.r0Hg     = model.r0Hg(:,121);
%        params.r0CoMg   = model.rCoM(:,121);
        params.mode     = -1;
    elseif i > 61
        params.r0Lg     = model.r01g(:,61);
        params.r0Rg     = model.r012g(:,61);
        params.r0Hg     = model.r0Hg(:,61);
%        params.r0CoMg   = model.rCoM(:,61);
        params.mode     =  1;
    else 
        params.r0Lg     = model.r01g(:,1);
        params.r0Rg     = model.r012g(:,1);
        params.r0Hg     = model.r0Hg(:,1);
%        params.r0CoMg   = model.rCoM(:,1);
        params.mode     = -1;
    end
    set(gca,'Color','#CCCCCC');
    [~, ~, HomegeneousTransforms] = k(model.q(:,i), params);

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
    % ONE
    r01 = HomegeneousTransforms.A01(1:3,4);
    plot3(r01(3), r01(1), r01(2), 'cx', 'LineWidth',2);         % J ONE
    % TWO
    r02 = HomegeneousTransforms.A02(1:3,4);
    plot3([r01(3) r02(3)], [r01(1) r02(1)], [r01(2) r02(2)],... % L TWO
    'k', 'LineWidth',2);
    plot3(r02(3), r02(1), r02(2) ,'cx', 'LineWidth',2);         % J TWO
    % THREE
    r03 = HomegeneousTransforms.A03(1:3,4);
    plot3([r02(3) r03(3)], [r02(1) r03(1)], [r02(2) r03(2)],... % L THREE
    'k', 'LineWidth',2);
    plot3(r03(3), r03(1), r03(2), 'cx', 'LineWidth',2);         % J THREE
    % FOUR
    r04 = HomegeneousTransforms.A04(1:3,4);
    plot3([r03(3) r04(3)], [r03(1) r04(1)], [r03(2) r04(2)],... % L FOUR
    'k', 'LineWidth',2);
    plot3(r04(3), r04(1), r04(2), 'cx', 'LineWidth',2);         % J FOUR
    % FIVE
    r05 = HomegeneousTransforms.A05(1:3,4);
    plot3([r04(3) r05(3)], [r04(1) r05(1)], [r04(2) r05(2)],... % L FIVE
    'k', 'LineWidth',2);
    plot3(r05(3), r05(1), r05(2), 'cx', 'LineWidth',2);         % J FIVE
    % SIX 
    r06 = HomegeneousTransforms.A06(1:3,4);
    plot3([r05(3) r06(3)], [r05(1) r06(1)], [r05(2) r06(2)],... % L SIX
    'k', 'LineWidth',2);
    plot3(r06(3), r06(1), r06(2), 'cx', 'LineWidth',2);         % J SIX

    % MID WAIST
    r0H = HomegeneousTransforms.A0H(1:3,4);
    plot3(r0H(3), r0H(1), r0H(2), 'mx', 'LineWidth',2);         % J MID WST

    % SEVEN 
    r07 = HomegeneousTransforms.A07(1:3,4);
    plot3([r06(3) r07(3)], [r06(1) r07(1)], [r06(2) r07(2)],... % L SEVEN
    'k', 'LineWidth',2);
    plot3(r07(3), r07(1), r07(2), 'bx', 'LineWidth',2);         % J SEVEN
    % EIGHT
    r08 = HomegeneousTransforms.A08(1:3,4);
    plot3([r07(3) r08(3)], [r07(1) r08(1)], [r07(2) r08(2)],... % L EIGHT
    'k', 'LineWidth',2);
    plot3(r08(3), r08(1), r08(2), 'bx', 'LineWidth',2);         % J EIGHT
    % NINE
    r09 = HomegeneousTransforms.A09(1:3,4);
    plot3([r08(3) r09(3)], [r08(1) r09(1)], [r08(2) r09(2)],... % L NINE
    'k', 'LineWidth',2);
    plot3(r09(3), r09(1), r09(2), 'bx', 'LineWidth',2);         % J NINE
    % TEN
    r010= HomegeneousTransforms.A010(1:3,4);
    plot3([r09(3) r010(3)], [r09(1) r010(1)], [r09(2) r010(2)],... % L TEN
    'k', 'LineWidth',2);
    plot3(r010(3), r010(1), r010(2), 'bx', 'LineWidth',2);         % J TEN
    % ELEVEN
    r011 = HomegeneousTransforms.A011(1:3,4);
    plot3([r010(3) r011(3)], [r010(1) r011(1)], [r010(2) r011(2)],... % L ELEVEN
    'k', 'LineWidth',2);
    plot3(r011(3), r011(1), r011(2), 'bx', 'LineWidth',2);            % J ELEVEN
    % TWELEVE
    r012 = HomegeneousTransforms.A012(1:3,4);
    plot3([r011(3) r012(3)], [r011(1) r012(1)], [r011(2) r012(2)],... % L TWELEVE
    'k', 'LineWidth',2);
    plot3(r012(3), r012(1), r012(2), 'bx', 'LineWidth',2);            % J TWELEVE

    % Plot the CoM
%     r0CoM = model.rCoM(:,i);
%     plot3(r0CoM(3),r0CoM(1),r0CoM(2), 'rx', 'LineWidth',1.5);

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
videoWriterObj.FrameRate = params.framerate;
open(videoWriterObj);                        
for i=1:length(IMAGE)
    frame = IMAGE(i);   % Convert from an Image to a Frame
    writeVideo(videoWriterObj, frame);
end
close(videoWriterObj);
