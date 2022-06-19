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
model.mode          = zeros(1,length(model.tspan)); % mode

% Physical Parameters
params.fibula       = 0.4;
params.femur        = 0.4;
params.HipWidth     = 0.2;
params.StepSize     = 0.2;
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
    params.mode         = -1; 
    i = 1;
    model.q(:,1)            = model.q0;
   [model.xe(:,1), ~, HTs]  = k(model.q0, i, model, params);
    model.r06g(:,1)         = HTs.A06(1:3,4);
    model.r01g(:,1)         = HTs.A01(1:3,4);
    model.r0Hg(:,1)         = HTs.A0H(1:3,4);

InitialFigure = figure(1);
    clf(InitialFigure)
    hold on
    grid on
    view(145,20);
    xlabel('{\bfZ} (metres)');
    ylabel('{\bfX} (metres)');
    zlabel('{\bfY} (metres)');
    set(gca,'Color','#EEEEEE');
    axis([-0.4,0.2, -1,1, 0,1]);
    title("2D Model - 3D View");
 
    % ZERO: Z,X,Y,   Z,  X,  Y
    quiver3(0,0,0, 0.5,0.0,0.0,'b','LineWidth',2); % Z
    quiver3(0,0,0, 0.0,0.5,0.0,'r','LineWidth',2); % X
    quiver3(0,0,0, 0.0,0.0,0.5,'g','LineWidth',2); % Y
    quiver3(0,0,0, model.r01g(3,1),model.r01g(1,1),0,'k');
    
    % MAIN COMPONENTS
    r01 = HTs.A01(1:3,4);
    plot3(r01(3), r01(1), r01(2), 'rx', 'LineWidth',3,'MarkerSize',10);
    r02 = HTs.A02(1:3,4);
    plot3([r01(3) r02(3)], [r01(1) r02(1)], [r01(2) r02(2)],...
        'k', 'LineWidth',2);
    plot3(r02(3), r02(1), r02(2) ,'rx', 'LineWidth',3,'MarkerSize',10);
    r03 = HTs.A03(1:3,4);
    plot3([r02(3) r03(3)], [r02(1) r03(1)], [r02(2) r03(2)],...
        'k', 'LineWidth',2);
    plot3(r03(3), r03(1), r03(2), 'rx', 'LineWidth',3,'MarkerSize',10);
    r0H = HTs.A0H(1:3,4);
    plot3(r0H(3), r0H(1), r0H(2), 'mx', 'LineWidth',3,'MarkerSize',10);
    r04 = HTs.A04(1:3,4);
    plot3([r03(3) r04(3)], [r03(1) r04(1)], [r03(2) r04(2)],...
        'k', 'LineWidth',2);
    plot3(r04(3), r04(1), r04(2), 'bx', 'LineWidth',3,'MarkerSize',10);
    r05 = HTs.A05(1:3,4);
    plot3([r04(3) r05(3)], [r04(1) r05(1)], [r04(2) r05(2)],...
        'k', 'LineWidth',2);
    plot3(r05(3), r05(1), r05(2), 'bx', 'LineWidth',3,'MarkerSize',10);
    r06 = HTs.A06(1:3,4);
    plot3([r05(3) r06(3)], [r05(1) r06(1)], [r05(2) r06(2)],...
        'k', 'LineWidth',2);
    plot3(r06(3), r06(1), r06(2), 'bx', 'LineWidth',3,'MarkerSize',10);

    legend({'+Z_0','+X_0','+Y_0','{r}^0_1 - \it{Link 1}',...
            'Joints 1 - 3','','','','','Mid Waist','','Joints 4 - 6'},...
            Location='west');

%% LOOP
% Initial Conditions
 model.q(:,1)              = model.q0;
[model.xe(:,1), ~, HTs]    = k(model.q0, 1, model, params);
% ... Thus, initial positons
 model.r06g(:,1)            = HTs.A06(1:3,4);
 model.r01g(:,1)            = HTs.A01(1:3,4);
 model.r0Hg(:,1)            = HTs.A0H(1:3,4);
% ... Thus, initial CoM 
 model.rCoM(:,1)            = rCoM(model.q(:,1),1,model,params);

% Trajectory Generation
[Q1,~,~] = trajectoryGeneration(1, model, 1:61, params);
 Q = Q1;

for i=2:61
    model.xe(:,i)   = [Q(:,i); zeros(3,1)];
    model.q(:,i)    = k_Inv(model.q(:,i-1), model.xe(:,i), i, model, params);
    [~, ~, HTs]     = k(model.q(:,i),(i-1),model, params);
    model.r01g(:,i) = HTs.A01(1:3,4);
    model.r06g(:,i) = HTs.A06(1:3,4);
    model.r0Hg(:,i) = HTs.A0H(1:3,4);
    model.rCoM(:,i) = rCoM(model.q(:,i),i,model,params);
    model.mode(:,i) = params.mode;
end

params.mode = 0;
% Trajectory Generation
[Q2,~,~] = trajectoryGeneration(61, model, 62:121, params);
 Q = [Q Q2];

for i=62:121
    model.xe(:,i)   = [Q(:,i); zeros(3,1)];
    model.q(:,i)    = k_Inv(model.q(:,i-1), model.xe(:,i), i, model, params);
    [~, ~, HTs]     = k(model.q(:,i),(i-1),model, params);
    model.r01g(:,i) = HTs.A01(1:3,4);
    model.r06g(:,i) = HTs.A06(1:3,4);
    model.r0Hg(:,i) = HTs.A0H(1:3,4);
    model.rCoM(:,i) = rCoM(model.q(:,i),i,model,params);
    model.mode(:,i) = params.mode;
end

params.mode = 1;
% Trajectory Generation
[Q3,~,~] = trajectoryGeneration(121, model, 122:181, params);
 Q = [Q Q3];

for i=122:181
    model.xe(:,i)   = [Q(:,i); zeros(3,1)];
    model.q(:,i)    = k_Inv(model.q(:,i-1), model.xe(:,i), i, model, params);
    [~, ~, HTs]     = k(model.q(:,i),(i-1),model, params);
    model.r01g(:,i) = HTs.A01(1:3,4);
    model.r06g(:,i) = HTs.A06(1:3,4);
    model.r0Hg(:,i) = HTs.A0H(1:3,4);
    model.rCoM(:,i) = rCoM(model.q(:,i),i,model,params);
    model.mode(:,i) = params.mode;
end

params.mode = 0;
% Trajectory Generation
[Q4,~,~] = trajectoryGeneration(181, model, 182:241, params);
 Q = [Q Q4];

for i=182:241
    model.xe(:,i)   = [Q(:,i); zeros(3,1)];
    model.q(:,i)    = k_Inv(model.q(:,i-1), model.xe(:,i), i, model, params);
    [~, ~, HTs]     = k(model.q(:,i),(i-1),model, params);
    model.r01g(:,i) = HTs.A01(1:3,4);
    model.r06g(:,i) = HTs.A06(1:3,4);
    model.r0Hg(:,i) = HTs.A0H(1:3,4);
    model.rCoM(:,i) = rCoM(model.q(:,i),i,model,params);
    model.mode(:,i) = params.mode;
end

toc % FINISH TIMING

%% Figures 
jointVariables = figure(2);
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

ANIMATION = figure(3);
for i=1:length(model.tspan)
    params.mode = model.mode(i);
    [~, ~, HTs]   = k(model.q(:,i), i, model, params);

    hold on
    grid on
    view(145,20);
    xlabel('{\bfZ} (metres)');
    ylabel('{\bfX} (metres)');
    zlabel('{\bfY} (metres)');
    set(gca,'Color','#EEEEEE');
    title("2D Model - 3D View");
    cla(ANIMATION)    % Clears Data but not Titles/Labels 
 
    % ZERO: Z,X,Y,   Z,  X,  Y
    quiver3(0,0,0, 0.5,0.0,0.0,'b','LineWidth',2); % Z
    quiver3(0,0,0, 0.0,0.5,0.0,'r','LineWidth',2); % X
    quiver3(0,0,0, 0.0,0.0,0.5,'g','LineWidth',2); % Y
    quiver3(0,0,0, model.r01g(3,i),model.r01g(1,i),model.r01g(2,i),'k');
    
    % MAIN COMPONENTS
    r01 = HTs.A01(1:3,4);
    plot3(r01(3), r01(1), r01(2), 'rx', 'LineWidth',3,'MarkerSize',10);
    r02 = HTs.A02(1:3,4);
    plot3([r01(3) r02(3)], [r01(1) r02(1)], [r01(2) r02(2)],...
        'k', 'LineWidth',2);
    plot3(r02(3), r02(1), r02(2) ,'rx', 'LineWidth',3,'MarkerSize',10);
    r03 = HTs.A03(1:3,4);
    plot3([r02(3) r03(3)], [r02(1) r03(1)], [r02(2) r03(2)],...
        'k', 'LineWidth',2);
    plot3(r03(3), r03(1), r03(2), 'rx', 'LineWidth',3,'MarkerSize',10);
    r0H = HTs.A0H(1:3,4);
    plot3(r0H(3), r0H(1), r0H(2), 'mx', 'LineWidth',3,'MarkerSize',10);
    r04 = HTs.A04(1:3,4);
    plot3([r03(3) r04(3)], [r03(1) r04(1)], [r03(2) r04(2)],...
        'k', 'LineWidth',2);
    plot3(r04(3), r04(1), r04(2), 'bx', 'LineWidth',3,'MarkerSize',10);
    r05 = HTs.A05(1:3,4);
    plot3([r04(3) r05(3)], [r04(1) r05(1)], [r04(2) r05(2)],...
        'k', 'LineWidth',2);
    plot3(r05(3), r05(1), r05(2), 'bx', 'LineWidth',3,'MarkerSize',10);
    r06 = HTs.A06(1:3,4);
    plot3([r05(3) r06(3)], [r05(1) r06(1)], [r05(2) r06(2)],...
        'k', 'LineWidth',2);
    plot3(r06(3), r06(1), r06(2), 'bx', 'LineWidth',3,'MarkerSize',10);

    legend({'+Z_0','+X_0','+Y_0','{r}^0_1 - \it{Link 1}',...
            'Joints 1 - 3','','','','','Mid Waist','','Joints 4 - 6'},...
            Location='west');

    %    [      MIN,      MAX, ...
    axis([     -0.4,      0.2, ... % Z
           r0H(1)-1, r0H(1)+1, ... % X
                  0,        1]);   % Y
    
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
