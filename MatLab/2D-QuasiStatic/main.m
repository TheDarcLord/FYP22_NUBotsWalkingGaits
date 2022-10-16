clear all
close all
clc 

%% Setup
params.framerate    = 10;
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
    model.q0 = [-pi/6;      % θ₁
               2*pi/6;      % θ₂
                -pi/6;      % θ₃
                 pi/6;      % θ₄
              -2*pi/6;      % θ₅
                 pi/6];     % θ₆

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
        axis equal
        view(145,20);
        xlabel('{\bfZ} (metres)');
        ylabel('{\bfX} (metres)');
        zlabel('{\bfY} (metres)');
        set(gca,'Color','#EEEEEE');
        axis([-0.4,0.2, -1,1, 0,1]);
        title("2D Model - 3D View");
        plotRobot(1,model,params);

    Q1 = trajGenStep(2:61,model,params);
        


%% MAIN LOOPs
tic % START TIMING
% Trajectory Generation
Q1 = trajGenStep(2:81,model,params);
Q  = [[0;0;0], Q1];



for i=2:81
    model.xe(:,i)    = [Q(:,i); zeros(3,1)];
    model.q(:,i)     = k_Inv(model.q(:,i-1), model.xe(:,i), i, model, params);
    [~, HTs]         = k(model.q(:,i),(i-1),model, params);
    model.rBLb(:,i)  = HTs.ABLb(1:3,4);
    model.rBRb(:,i)  = HTs.ABRb(1:3,4);
    model.rBHb(:,i)  = HTs.AbH(1:3,4);
    model.rCoMb(:,i) = rCoM(model.q(:,i),i,model,params);
    model.mode(:,i)  = params.mode;
    
    [model.rBRb(1,i) model.rBLb(1,i) model.rCoMb(1,i)]

    ANIMATION = figure(1);
        clf(ANIMATION)
        hold on
        grid on
        axis equal
        view(145,20);
        xlabel('{\bfZ} (metres)');
        ylabel('{\bfX} (metres)');
        zlabel('{\bfY} (metres)');
        set(gca,'Color','#EEEEEE');
        axis([-0.4,0.2, -1,1, 0,1]);
        title("2D Model - 3D View");
        plotRobot(i,model,params);
        plot3(Q(3,:),Q(1,:),Q(2,:),'b-');
        drawnow
        pause(0.01)
end

params.mode = 1;
% Trajectory Generation
Q2 = trajGenStep(82:161,model,params);
Q = [Q Q2];

 ANIMATION = figure(1);
        clf(ANIMATION)
        hold on
        grid on
        axis equal
        view(145,20);
        xlabel('{\bfZ} (metres)');
        ylabel('{\bfX} (metres)');
        zlabel('{\bfY} (metres)');
        set(gca,'Color','#EEEEEE');
        axis([-0.4,0.2, -1,1, 0,1]);
        title("2D Model - 3D View");
        plotRobot(81,model,params);
        plot3(Q(3,:),Q(1,:),Q(2,:),'b-');
        drawnow
        pause(0.01)

for i=82:161
    model.xe(:,i)   = [Q(:,i); zeros(3,1)];
    model.q(:,i)    = k_Inv(model.q(:,i-1), model.xe(:,i), i, model, params);
    [~, HTs]        = k(model.q(:,i),(i-1),model, params);
    model.rBLb(:,i) = HTs.ABLb(1:3,4);
    model.rBRb(:,i) = HTs.ABRb(1:3,4);
    model.rBHb(:,i) = HTs.AbH(1:3,4);
    model.rCoMb(:,i) = rCoM(model.q(:,i),i,model,params);
    model.mode(:,i) = params.mode;

    ANIMATION = figure(1);
        clf(ANIMATION)
        hold on
        grid on
        axis equal
        view(145,20);
        xlabel('{\bfZ} (metres)');
        ylabel('{\bfX} (metres)');
        zlabel('{\bfY} (metres)');
        set(gca,'Color','#EEEEEE');
        axis([-0.4,0.2, -1,1, 0,1]);
        title("2D Model - 3D View");
        plotRobot(i,model,params);
        plot3(Q(3,:),Q(1,:),Q(2,:),'b-');
        drawnow
        pause(0.01)
end

params.mode = -1;
% Trajectory Generation
Q3 = trajGenStep(162:241, model, params);
Q = [Q Q3];

ANIMATION = figure(1);
        clf(ANIMATION)
        hold on
        grid on
        axis equal
        view(145,20);
        xlabel('{\bfZ} (metres)');
        ylabel('{\bfX} (metres)');
        zlabel('{\bfY} (metres)');
        set(gca,'Color','#EEEEEE');
        axis([-0.4,0.2, -1,1, 0,1]);
        title("2D Model - 3D View");
        plotRobot(161,model,params);
        plot3(Q(3,:),Q(1,:),Q(2,:),'b-');
        drawnow
        pause(0.01)

for i=162:241
    model.xe(:,i)   = [Q(:,i); zeros(3,1)];
    model.q(:,i)    = k_Inv(model.q(:,i-1), model.xe(:,i), i, model, params);
    [~, HTs]     = k(model.q(:,i),(i-1),model, params);
    model.rBLb(:,i) = HTs.ABLb(1:3,4);
    model.rBRb(:,i) = HTs.ABRb(1:3,4);
    model.rBHb(:,i) = HTs.AbH(1:3,4);
    model.rCoMb(:,i) = rCoM(model.q(:,i),i,model,params);
    model.mode(:,i) = params.mode;
end

% params.mode = 1;
% % Trajectory Generation
% [Q4,~,~] = trajectoryGeneration(181, model, 182:241, params);
%  Q = [Q Q4];
% 
% for i=182:241
%     model.xe(:,i)   = [Q(:,i); zeros(3,1)];
%     model.q(:,i)    = k_Inv(model.q(:,i-1), model.xe(:,i), i, model, params);
%     [~, HTs]     = k(model.q(:,i),(i-1),model, params);
%     model.rBLb(:,i) = HTs.ABLb(1:3,4);
%     model.rBRb(:,i) = HTs.ABRb(1:3,4);
%     model.rBHb(:,i) = HTs.AbH(1:3,4);
%     model.rCoMb(:,i) = rCoM(model.q(:,i),i,model,params);
%     model.mode(:,i) = params.mode;
% end

toc % FINISH TIMING

%% Figures 
% jointVariables = figure(2);
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

ANIMATION = figure(3);
for i=1:length(model.tspan)
    params.mode = model.mode(i);

    clf(ANIMATION)
    hold on
    grid on
    view(145,20);
    xlabel('{\bfZ} (metres)');
    ylabel('{\bfX} (metres)');
    zlabel('{\bfY} (metres)');
    set(gca,'Color','#EEEEEE');
    axis([-0.4,0.2, -1,1, 0,1]);
    title("2D Model - 3D View");
 
    plotRobot(i,model,params);

    plot3(model.rCoMb(3,i)*[1 1], ...
          model.rCoMb(1,i)*[1 1], ...
          model.rCoMb(2,i)*[1 0],'m:','LineWidth',2)

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
videoWriterObj.FrameRate = params.framerate * 2;
open(videoWriterObj);                        
for i=1:length(IMAGE)
    frame = IMAGE(i);   % Convert from an Image to a Frame
    writeVideo(videoWriterObj, frame);
end
close(videoWriterObj);
