clear all
close all
clc 

%% Setup
params.framerate    = 10;
model.tspan         = 0:(1 / params.framerate):24;

model.q             = zeros(6,length(model.tspan)); % q     [θ₁θ₂θ₃θ₄θ₅θ₆]ᵀ
model.xe            = zeros(9,length(model.tspan)); % xe    [XYZϕθΨ]ᵀ
model.rGBg          = zeros(3,length(model.tspan)); %       [XYZ]ᵀ
model.rBLb          = zeros(3,length(model.tspan)); %       [XYZ]ᵀ
model.rBRb          = zeros(3,length(model.tspan)); %       [XYZ]ᵀ
model.rCoMb         = zeros(3,length(model.tspan)); % rCoMb [XYZ]ᵀ
model.mode          = zeros(1,length(model.tspan)); % mode

% Physical Parameters
params.StepSize     = 0.25;
params.fibula       = 0.4;  % Lower Leg
params.femur        = 0.4;  % Upper Leg
params.tarsal       = 0.05; % Ankle Height
params.HipWidth     = 0.2;
params.mode         = 0;    % BOTH  FIXED
%                    -1;    % LEFT  FIXED
%                     1;    % RIGHT FIXED
% Masses
params.mass.femur   = 1;    % Thigh Bone
params.mass.fibula  = 1;    % Paired with `tibia`
params.mass.joint   = 0.5;  % Knee Bone / Joints / Ankles
params.mass.pelvis  = 0.7;  % Waist
params.mass.foot    = 0.1;  % Foot

%% Initial Position & Orientation & Conditions & Figure
    i = 1;
    params.mode = -1;
    model.q0 = [pi/6; -2*pi/6; pi/6; ... % [θ₁,θ₂,θ₃, ...
                pi/6; -2*pi/6; pi/6];    %  θ₄,θ₅,θ₆]ᵀ
    % Initial -> Both feet on Ground
   [model.rGBg(:,i), ...
    model.rBLb(:,i), ... 
    model.rBRb(:,i), HTs] = k_init(model.q0, params);
    model.rCoMb(:,i)      = rCoM(model.q0,i,model,params);

INTITAL_FIGURE = figure(1);
    clf(INTITAL_FIGURE)
    title("2D Model - LOCAL");
    plotModel(HTs,model,1);

    % TRAJECTORY CHECK
    [Q,~,~] = trajectoryGeneration(1, model, 1:61, params);
    plot3(Q(1,:),Q(2,:),Q(3,:),'k:','LineWidth',2);

%% MAIN LOOPs
tic % START TIMING
% Trajectory Generation
[Q1,~,~] = trajectoryGeneration(1, model, 1:61, params);
 Q = Q1;

for i=2:61
    model.xe(:,i)   = [Q(:,i);0;0;0;model.rBLb(:,i-1)];
    model.q(:,i)    = k_Inv(model.q(:,i-1), model.xe(:,i), i, model, params);
    [~, HTs]        = kSLOW(model.q(:,i),(i-1),model, params);
    model.rBLb(:,i) = HTs.AGE_L(1:3,4);
    model.rBRb(:,i) = HTs.AGE_R(1:3,4);
    model.rGBg(:,i) = HTs.AGH(1:3,4);
    model.rCoMb(:,i) = rCoM(model.q(:,i),i,model,params);
    model.mode(:,i) = params.mode;
end

% params.mode = 1;
% % Trajectory Generation
% [Q2,~,~] = trajectoryGeneration(61, model, 62:121, params);
%  Q = [Q Q2];
% 
% for i=62:121
%     model.xe(:,i)   = [Q(:,i);0;0;0;model.rBRb(:,i-1)];
%     model.q(:,i)    = k_Inv(model.q(:,i-1), model.xe(:,i), i, model, params);
%     [~, HTs]        = kSLOW(model.q(:,i),(i-1),model, params);
%     model.rBLb(:,i) = HTs.AGE_L(1:3,4);
%     model.rBRb(:,i) = HTs.AGE_R(1:3,4);
%     model.rGBg(:,i) = HTs.AGH(1:3,4);
%     model.rCoMb(:,i) = rCoM(model.q(:,i),i,model,params);
%     model.mode(:,i) = params.mode;
% end


% params.mode = 0;
% % Trajectory Generation
% [Q3,~,~] = trajectoryGeneration(121, model, 122:181, params);
%  Q = [Q Q3];
% 
% for i=122:181
%     model.xe(:,i)   = [Q(:,i); zeros(3,1)];
%     model.q(:,i)    = k_Inv(model.q(:,i-1), model.xe(:,i), i, model, params);
%     [~, HTs]     = k(model.q(:,i),(i-1),model, params);
%     model.rBLb(:,i) = HTs.ABLb(1:3,4);
%     model.rBRb(:,i) = HTs.ABRb(1:3,4);
%     model.rBHb(:,i) = HTs.AbH(1:3,4);
%     model.rCoMb(:,i) = rCoM(model.q(:,i),i,model,params);
%     model.mode(:,i) = params.mode;
% end
% 
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
    [~, HTs] = kSLOW(model.q(:,i), i, model, params);
    clf(ANIMATION)
    plotModel(HTs,model,i)

    drawnow
end

% %% VIDEO
% videoWriterObj           = VideoWriter('2D_Step.mp4','MPEG-4');
% videoWriterObj.FrameRate = params.framerate * 2;
% open(videoWriterObj);                        
% for i=1:length(IMAGE)
%     frame = IMAGE(i);   % Convert from an Image to a Frame
%     writeVideo(videoWriterObj, frame);
% end
% close(videoWriterObj);
