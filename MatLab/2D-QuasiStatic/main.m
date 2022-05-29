clear all
close all
clc 

tic % START TIMING

%% Setup
params.framerate    = 10;
model.tspan         = 0:(1 / params.framerate):24;

model.q             = zeros(6,length(model.tspan)); % q    [θ₁θ₂θ₃θ₄θ₅θ₆]ᵀ
model.xe            = zeros(6,length(model.tspan)); % xe   [XYZϕθΨ]ᵀ
model.r01g          = zeros(3,length(model.tspan)); % A01  [XYZ]ᵀ
model.r06g          = zeros(3,length(model.tspan)); % A06  [XYZ]ᵀ
model.r0Hg          = zeros(3,length(model.tspan)); % A0H  [XYZ]ᵀ
model.rCoM          = zeros(3,length(model.tspan)); % rCoM [XYZ]ᵀ

% Physical Parameters
params.fibula       = 0.4;
params.femur        = 0.4;
params.HipWidth     = 0.2;
params.StepSize     = 0.2;
params.r0Lg         = [0.2; 0; -0.1];  % Left  Position from 0rigin in Global
params.r0Hg         = zeros(3,1);  % Waist Position from 0rigin in Global
params.r0Rg         = zeros(3,1);  % Right Position from 0rigin in Global
params.r0CoMg       = zeros(3,1);  % CoM   Position from 0rigin in Global
params.mode         = -1;          % LEFT  FIXED - FKM T16
%                      0;          % BOTH  FIXED - FKM T1H T6H
%                      1;          % RIGHT FIXED - FKM T61
% Masses
params.mass.femur   = 1;    % Thigh Bone
params.mass.fibula  = 1;    % Paired with `tibia`
params.mass.joint   = 0.5;  % Knee Bone / Joints / Ankles
params.mass.pelvis  = 0.7;  % Waist

%% Initial Position & Orientation

model.q0 = [-pi/6;      % θ₁
           2*pi/6;      % θ₂
            -pi/6;      % θ₃
             pi/6;      % θ₄
          -2*pi/6;      % θ₅
             pi/6];     % θ₆

%% Initial Figure
    model.q( :,1)              = model.q0;
   [model.xe(:,1), ~, HTs]     = k(model.q0, params);
    % ... Thus, initial positons
    model.r06g(:,1)            = HTs.A06(1:3,4);
    model.r01g(:,1)            = HTs.A01(1:3,4);
    model.r0Hg(:,1)            = HTs.A0H(1:3,4);
    i = 1;
InitialFigure = figure(1);
    clf(InitialFigure)
    subplot(1,2,1)
        hold on
        grid on
        set(gca,'Color','#CCCCCC');
        view(145,20);
        [~, ~, HomegeneousTransforms] = k(model.q(:,i), params);
        % ZERO:   Z      X      Y
        plot3([0 1], [0 0], [0 0],'r', 'LineWidth',3); % Z
        plot3([0 0], [0 1], [0 0], 'LineWidth',3,'Color','#379203'); % X
        plot3([0 0], [0 0], [0 1],'b', 'LineWidth',3); % Y
        quiver3(0,0,0,model.r01g(3,1),model.r01g(1,1),0,'LineWidth',2,'Color','k','ShowArrowHead','on')
        title("2D Model - 3D View",'FontSize',18);
        xlabel('{\bfZ} (metres)');
        ylabel('{\bfX} (metres)');
        zlabel('{\bfY} (metres)');
        % MAIN COMPONENTS
        r01 = HomegeneousTransforms.A01(1:3,4);
        plot3(r01(3), r01(1), r01(2), 'rx', 'LineWidth',3,'MarkerSize',10);
        r02 = HomegeneousTransforms.A02(1:3,4);
        plot3([r01(3) r02(3)], [r01(1) r02(1)], [r01(2) r02(2)],...
            'k', 'LineWidth',2);
        plot3(r02(3), r02(1), r02(2) ,'rx', 'LineWidth',3,'MarkerSize',10);
        r03 = HomegeneousTransforms.A03(1:3,4);
        plot3([r02(3) r03(3)], [r02(1) r03(1)], [r02(2) r03(2)],...
            'k', 'LineWidth',2);
        plot3(r03(3), r03(1), r03(2), 'rx', 'LineWidth',3,'MarkerSize',10);
        r0H = HomegeneousTransforms.A0H(1:3,4);
        plot3(r0H(3), r0H(1), r0H(2), 'mx', 'LineWidth',3,'MarkerSize',10);
        r04 = HomegeneousTransforms.A04(1:3,4);
        plot3([r03(3) r04(3)], [r03(1) r04(1)], [r03(2) r04(2)],...
            'k', 'LineWidth',2);
        plot3(r04(3), r04(1), r04(2), 'bx', 'LineWidth',3,'MarkerSize',10);
        r05 = HomegeneousTransforms.A05(1:3,4);
        plot3([r04(3) r05(3)], [r04(1) r05(1)], [r04(2) r05(2)],...
            'k', 'LineWidth',2);
        plot3(r05(3), r05(1), r05(2), 'bx', 'LineWidth',3,'MarkerSize',10);
        r06 = HomegeneousTransforms.A06(1:3,4);
        plot3([r05(3) r06(3)], [r05(1) r06(1)], [r05(2) r06(2)],...
            'k', 'LineWidth',2);
        plot3(r06(3), r06(1), r06(2), 'bx', 'LineWidth',3,'MarkerSize',10);

        legend({'+Z_0','+X_0','+Y_0','{r}^0_1 - \it{Link 1}',...
                'Joints 1 - 3','','','','','Mid Waist','','Joints 4 - 6'},...
                'FontSize',16,Location='west');
        
        axis([-0.4,0.2, -1,1, 0,1]);

    subplot(1,2,2)
        hold on
        grid on
        set(gca,'Color','#CCCCCC');
        view(90,0);

        % ZERO:   Z      X      Y
        plot3([0 1], [0 0], [0 0],'r', 'LineWidth',3); % Z
        plot3([0 0], [0 1], [0 0], 'LineWidth',3,'Color','#379203'); % X
        plot3([0 0], [0 0], [0 1],'b', 'LineWidth',3); % Y
        legend({'+Z_0','+X_0','+Y_0'},'Autoupdate','off','Location','west');
        title("2D Model - 2D View",'FontSize',18);
        xlabel('{\bfZ} (metres)');
        ylabel('{\bfX} (metres)');
        zlabel('{\bfY} (metres)');
        r01 = HomegeneousTransforms.A01(1:3,4);
        plot3(r01(3), r01(1), r01(2), 'rx', 'LineWidth',3,'MarkerSize',8);
        r02 = HomegeneousTransforms.A02(1:3,4);
        plot3([r01(3) r02(3)], [r01(1) r02(1)], [r01(2) r02(2)],...
            'k', 'LineWidth',2);
        plot3(r02(3), r02(1), r02(2) ,'rx', 'LineWidth',3,'MarkerSize',8);
        r03 = HomegeneousTransforms.A03(1:3,4);
        plot3([r02(3) r03(3)], [r02(1) r03(1)], [r02(2) r03(2)],...
            'k', 'LineWidth',2);
        plot3(r03(3), r03(1), r03(2), 'rx', 'LineWidth',3,'MarkerSize',8);
        r0H = HomegeneousTransforms.A0H(1:3,4);
        plot3(r0H(3), r0H(1), r0H(2), 'mx', 'LineWidth',3,'MarkerSize',8);
        r04 = HomegeneousTransforms.A04(1:3,4);
        plot3([r03(3) r04(3)], [r03(1) r04(1)], [r03(2) r04(2)],...
            'k', 'LineWidth',2);
        plot3(r04(3), r04(1), r04(2), 'bx', 'LineWidth',3,'MarkerSize',8);
        r05 = HomegeneousTransforms.A05(1:3,4);
        plot3([r04(3) r05(3)], [r04(1) r05(1)], [r04(2) r05(2)],...
            'k', 'LineWidth',2);
        plot3(r05(3), r05(1), r05(2), 'bx', 'LineWidth',3,'MarkerSize',8);
        r06 = HomegeneousTransforms.A06(1:3,4);
        plot3([r05(3) r06(3)], [r05(1) r06(1)], [r05(2) r06(2)],...
            'k', 'LineWidth',2);
        plot3(r06(3), r06(1), r06(2), 'bx', 'LineWidth',3,'MarkerSize',8);

        legend({'+Z_0','+X_0','+Y_0',...
                'Joints 1 - 3','','','','','Mid Waist','','Joints 4 - 6'},...
                'FontSize',16,Location='west');

        axis([-0.4,0.2, -1,1, 0,1]);

%% LOOP
% Initial Conditions
 model.q( :,1)              = model.q0;
[model.xe(:,1), ~, HTs]     = k(model.q0, params);
% ... Thus, initial positons
 model.r06g(:,1)            = HTs.A06(1:3,4);
 model.r01g(:,1)            = HTs.A01(1:3,4);
 model.r0Hg(:,1)            = HTs.A0H(1:3,4);
% ... Store in Params... Maybe Parse Model ?
 params.r0Lg                = model.r01g(:,1);
 params.r0Rg                = model.r06g(:,1);
 params.r0Hg                = model.r0Hg(:,1);
% ... Thus, initial CoM 
 model.rCoM(:,1)            = rCoM(model.q(:,1),params);
 params.r0CoMg              = model.rCoM(:,1);

% Trajectory Generation
[Q1,~,~] = trajectoryGeneration(model, 1:61,params);

for i=2:61
    model.xe(:,i)   = [Q1(:,i); zeros(3,1)];
    model.q(:,i)    = k_Inv(model.q(:,i-1), model.xe(:,i), params);
    [~, ~, HTs]     = k(model.q(:,i), params);
    model.r01g(:,i) = HTs.A01(1:3,4);
     params.r0Lg    = model.r01g(:,i);
    model.r06g(:,i) = HTs.A06(1:3,4);
     params.r0Rg    = model.r06g(:,i);
    model.r0Hg(:,i) = HTs.A0H(1:3,4);
     params.r0Hg    = model.r0Hg(:,i);
    model.rCoM(:,i) = rCoM(model.q(:,i),params);
     params.r0CoMg  = model.rCoM(:,i);
end

params.mode = 0;
% Trajectory Generation
[Q2,~,~] = trajectoryGeneration(model, 62:121,params); 

for i=62:121
    model.xe(:,i)   = [Q2(:,i-61); zeros(3,1)];
    model.q(:,i)    = k_Inv(model.q(:,i-1), model.xe(:,i), params);
    [~, ~, HTs]     = k(model.q(:,i), params);
    model.r01g(:,i) = HTs.A01(1:3,4);
     params.r0Lg    = model.r01g(:,i);
    model.r06g(:,i) = HTs.A06(1:3,4);
     params.r0Rg    = model.r06g(:,i);
    model.r0Hg(:,i) = HTs.A0H(1:3,4);
     params.r0Hg    = model.r0Hg(:,i);
    model.rCoM(:,i) = rCoM(model.q(:,i),params);
     params.r0CoMg  = model.rCoM(:,i);
end

params.mode = 1;
% Trajectory Generation
[Q3,~,~] = trajectoryGeneration(model, 122:181,params); 

for i=122:181
    model.xe(:,i)   = [Q3(:,i-121); zeros(3,1)];
    model.q(:,i)    = k_Inv(model.q(:,i-1), model.xe(:,i), params);
    [~, ~, HTs]     = k(model.q(:,i), params);
    model.r01g(:,i) = HTs.A01(1:3,4);
     params.r0Lg    = model.r01g(:,i);
    model.r06g(:,i) = HTs.A06(1:3,4);
     params.r0Rg    = model.r06g(:,i);
    model.r0Hg(:,i) = HTs.A0H(1:3,4);
     params.r0Hg    = model.r0Hg(:,i);
    model.rCoM(:,i) = rCoM(model.q(:,i),params);
     params.r0CoMg  = model.rCoM(:,i);
end

params.mode = 0;
% Trajectory Generation
[Q4,~,~] = trajectoryGeneration(model, 182:241,params); 

for i=182:241
    model.xe(:,i)   = [Q4(:,i-181); zeros(3,1)];
    model.q(:,i)    = k_Inv(model.q(:,i-1), model.xe(:,i), params);
    [~, ~, HTs]     = k(model.q(:,i), params);
    model.r01g(:,i) = HTs.A01(1:3,4);
     params.r0Lg    = model.r01g(:,i);
    model.r06g(:,i) = HTs.A06(1:3,4);
     params.r0Rg    = model.r06g(:,i);
    model.r0Hg(:,i) = HTs.A0H(1:3,4);
     params.r0Hg    = model.r0Hg(:,i);
    model.rCoM(:,i) = rCoM(model.q(:,i),params);
     params.r0CoMg  = model.rCoM(:,i);
end

toc % FINISH TIMING

%% Figures 
figure('Name','Joint Variables, q(t)')
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
    title('All Joint Variables: {\itθ}_{1-6}({\itt})','FontSize',12);
    legend('θ₁','θ₂','θ₃', 'θ₄','θ₅','θ₆');

figure('Name','Foot,Waist,CoM Movement')
    hold on
    grid on
    title('Foot & Waist & CoM Trajectories','FontSize',12);
    set(gca,'Color','#CCCCCC');
    plot3([0 1], [0 0], [0 0],'r', 'LineWidth',0.5); % Z
    plot3([0 0], [0 1], [0 0],'g', 'LineWidth',0.5); % X
    plot3([0 0], [0 0], [0 1],'b', 'LineWidth',0.5); % Y
    plot3(model.r01g(3,:),model.r01g(1,:),model.r01g(2,:),...
        'c-','LineWidth',2);
    plot3(model.r06g(3,:),model.r06g(1,:),model.r06g(2,:),...
        'g-','LineWidth',2);
    plot3(model.r0Hg(3,:),model.r0Hg(1,:),model.r0Hg(2,:),...
        'm-','LineWidth',2);
    plot3(model.rCoM(3,:),model.rCoM(1,:),model.rCoM(2,:),...
        'r-','LineWidth',1);

    legend('+Z','+X','+Y','Left','Right','Waist', 'CoM');
    axis([-(params.HipWidth+0.2) 0.2 min(model.r0Hg(1,:))-0.1 max(model.r0Hg(1,:))+0.2 0 1]);
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
    if i > 181
        params.r0Lg     = model.r01g(:,181);
        params.r0Rg     = model.r06g(:,181);
        params.r0Hg     = model.r0Hg(:,181);
        params.r0CoMg   = model.rCoM(:,181);
        params.mode     =  0;
    elseif i > 121
        params.r0Lg     = model.r01g(:,121);
        params.r0Rg     = model.r06g(:,121);
        params.r0Hg     = model.r0Hg(:,121);
        params.r0CoMg   = model.rCoM(:,121);
        params.mode     =  1;
    elseif i > 61
        params.r0Lg     = model.r01g(:,61);
        params.r0Rg     = model.r06g(:,61);
        params.r0Hg     = model.r0Hg(:,61);
        params.r0CoMg   = model.rCoM(:,61);
        params.mode     =  0;
    else 
        params.r0Lg     = model.r01g(:,1);
        params.r0Rg     = model.r06g(:,1);
        params.r0Hg     = model.r0Hg(:,1);
        params.r0CoMg   = model.rCoM(:,1);
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
    % MID WAIST
    r0H = HomegeneousTransforms.A0H(1:3,4);
    plot3(r0H(3), r0H(1), r0H(2), 'mx', 'LineWidth',2);         % J MID WST
    % FOUR
    r04 = HomegeneousTransforms.A04(1:3,4);
    plot3([r03(3) r04(3)], [r03(1) r04(1)], [r03(2) r04(2)],... % L FOUR
    'k', 'LineWidth',2);
    plot3(r04(3), r04(1), r04(2), 'gx', 'LineWidth',2);         % J FOUR
    % FIVE
    r05 = HomegeneousTransforms.A05(1:3,4);
    plot3([r04(3) r05(3)], [r04(1) r05(1)], [r04(2) r05(2)],... % L FIVE
    'k', 'LineWidth',2);
    plot3(r05(3), r05(1), r05(2), 'gx', 'LineWidth',2);         % J FIVE
    % SIX 
    r06 = HomegeneousTransforms.A06(1:3,4);
    plot3([r05(3) r06(3)], [r05(1) r06(1)], [r05(2) r06(2)],... % L SIX
    'k', 'LineWidth',2);
    plot3(r06(3), r06(1), r06(2), 'gx', 'LineWidth',2);         % J SIX

    % Plot the CoM
    r0CoM = model.rCoM(:,i);
    plot3(r0CoM(3),r0CoM(1),r0CoM(2), 'ro', 'LineWidth',1.5);
    plot3(r0CoM(3),r0CoM(1),0, 'rx', 'LineWidth',2);

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
videoWriterObj           = VideoWriter('2D_Step.mp4','MPEG-4');
videoWriterObj.FrameRate = params.framerate;
open(videoWriterObj);                        
for i=1:length(IMAGE)
    frame = IMAGE(i);   % Convert from an Image to a Frame
    writeVideo(videoWriterObj, frame);
end
close(videoWriterObj);
