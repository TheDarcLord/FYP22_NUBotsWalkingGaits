clear all
close all
clc 

%% Setup
params.framerate    = 40;
model.tspan         = 0:(1 / params.framerate):24;

model.q             = zeros(6,length(model.tspan)); % q     [θ₁θ₂θ₃θ₄θ₅θ₆]ᵀ
model.xe            = zeros(6,length(model.tspan)); % xe    [XYZϕθΨ]ᵀ
model.rBLb          = zeros(3,length(model.tspan)); %       [XYZ]ᵀ
model.rBRb          = zeros(3,length(model.tspan)); %       [XYZ]ᵀ
model.rBHb          = zeros(3,length(model.tspan)); %       [XYZ]ᵀ
model.rCoMb         = zeros(3,length(model.tspan)); % rCoMb [XYZ]ᵀ
model.mode          = zeros(1,length(model.tspan)); % mode

% Physical Parameters
params.fibula       = 0.4;
params.femur        = 0.4;
params.tarsal       = 0.05;              
params.HipWidth     = 0.2;
params.StepSize     = 0.25;
params.StepHeight   = 0.1;
params.mode         = -1;          % LEFT  FIXED - FKM T16
%                      0;          % BOTH  FIXED - FKM T1H T6H
%                      1;          % RIGHT FIXED - FKM T61
% Masses
params.mass.femur   = 1;    % Thigh Bone
params.mass.fibula  = 1;    % Paired with `tibia`
params.mass.joint   = 0.5;  % Knee Bone / Joints / Ankles
params.mass.pelvis  = 0.7;  % Waist
params.mass.foot    = 0.1;  % Foot

%% Initial Position & Orientation & Conditions
    model.q0 = [-pi/8;      % θ₁
               2*pi/8;      % θ₂
                -pi/8;      % θ₃
                 pi/8;      % θ₄
              -2*pi/8;      % θ₅
                 pi/8];     % θ₆

    % Initial Conditions
    model.q(:,1)        = model.q0;
   [model.xe(:,1), HTs] = k(model.q0, 1, model, params);
    % ... Thus, initial positons
    model.rBLb(:,1)     = HTs.ABLb(1:3,4);
    model.rBRb(:,1)     = HTs.ABRb(1:3,4);
    model.rBHb(:,1)     = HTs.AbH(1:3,4);
    % ... Thus, initial CoM 
    model.rCoMb(:,1) = rCoM(model.q(:,1),1,model,params);
    model.mode(1,1)  = params.mode;

    ANIMATION = figure(1);
        clf(ANIMATION)
        hold on
        grid on
        grid("minor");
        axis equal
        view(90,0);
        xlabel('{\bfZ} (metres)');
        ylabel('{\bfX} (metres)');
        zlabel('{\bfY} (metres)');
        set(gca,'Color','#DDDDDD');
        title("2D Model - 3D View");
        plotRobot(1,model,params);

    Q1 = trajGenStep(2:321,model,params);
        plot3(Q1(3,:),Q1(1,:),Q1(2,:),'Color','#228B22','LineWidth',2);

%% MAIN LOOPs
tic % START TIMING
% Trajectory Generation
Q1 = trajGenStep(2:321,model,params);
Q  = [[0;0;0], Q1];

for i=2:321
    model.xe(:,i)    = [Q(:,i); zeros(3,1)];
    model.q(:,i)     = k_Inv(model.q(:,i-1), model.xe(:,i), i, model, params);
    [~, HTs]         = k(model.q(:,i),(i-1),model, params);
    model.rBLb(:,i)  = HTs.ABLb(1:3,4);
    model.rBRb(:,i)  = HTs.ABRb(1:3,4);
    model.rBHb(:,i)  = HTs.AbH(1:3,4);
    model.rCoMb(:,i) = rCoM(model.q(:,i),i,model,params);
    model.mode(:,i)  = params.mode;
end

params.mode = 1;
% Trajectory Generation
Q2 = trajGenStep(322:641,model,params);
Q = [Q Q2];

for i=322:641
    model.xe(:,i)   = [Q(:,i); zeros(3,1)];
    model.q(:,i)    = k_Inv(model.q(:,i-1), model.xe(:,i), i, model, params);
    [~, HTs]        = k(model.q(:,i),(i-1),model, params);
    model.rBLb(:,i) = HTs.ABLb(1:3,4);
    model.rBRb(:,i) = HTs.ABRb(1:3,4);
    model.rBHb(:,i) = HTs.AbH(1:3,4);
    model.rCoMb(:,i) = rCoM(model.q(:,i),i,model,params);
    model.mode(:,i) = params.mode;
end

params.mode = -1;
% Trajectory Generation
Q3 = trajGenStep(642:961, model, params);
Q = [Q Q3];

for i=642:961
    model.xe(:,i)   = [Q(:,i); zeros(3,1)];
    model.q(:,i)    = k_Inv(model.q(:,i-1), model.xe(:,i), i, model, params);
    [~, HTs]     = k(model.q(:,i),(i-1),model, params);
    model.rBLb(:,i) = HTs.ABLb(1:3,4);
    model.rBRb(:,i) = HTs.ABRb(1:3,4);
    model.rBHb(:,i) = HTs.AbH(1:3,4);
    model.rCoMb(:,i) = rCoM(model.q(:,i),i,model,params);
    model.mode(:,i) = params.mode;
end

toc % FINISH TIMING

%% Figures
ANIMATION = figure(3);
    
for i=1:length(model.tspan)
    clf(ANIMATION)
    hold on
    grid on
    grid("minor");
    axis equal
    view(140,30);
    xlabel('{\bfZ} (metres)');
    ylabel('{\bfX} (metres)');
    zlabel('{\bfY} (metres)');
    set(gca,'Color','#DDDDDD');
    title("2D Model - 3D View");

    params.mode = model.mode(i);
    plotRobot(i,model,params);
    plot3(model.rCoMb(3,i), ...
          model.rCoMb(1,i), ...
          model.rCoMb(2,i),'ro','LineWidth',2,'MarkerSize',10)
    plot3(model.rCoMb(3,i), ...
          model.rCoMb(1,i), ...
          0,'rx','LineWidth',2,'MarkerSize',10)

%     legend({'+Z_0','+X_0','+Y_0','{r}^B_0 - \it{Link 0}',...
%             'Joints (1 - 3)','','','','','Mid Waist','',...
%             'Joints (4 - 6)','','','','','','','', ...
%             'End Effector', 'CoM'},...
%             Location='west');
    
    %    [      MIN,      MAX, ...
    
    
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
