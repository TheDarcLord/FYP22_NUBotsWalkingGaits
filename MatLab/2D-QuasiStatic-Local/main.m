clear all
close all
clc 

%% Setup
params.framerate    = 10;
model.tspan         = 0:(1 / params.framerate):24;

model.q             = zeros(6,length(model.tspan)); % q     [θ₁θ₂θ₃θ₄θ₅θ₆]ᵀ
model.xe            = zeros(6,length(model.tspan)); % xe    [XYZϕθΨ]ᵀ
model.rGBg          = zeros(3,length(model.tspan)); %       [XYZ]ᵀ
model.rBLb          = zeros(3,length(model.tspan)); %       [XYZ]ᵀ
model.rBRb          = zeros(3,length(model.tspan)); %       [XYZ]ᵀ
model.rCoMg         = zeros(3,length(model.tspan)); % rCoMb [XYZ]ᵀ
model.mode          = zeros(1,length(model.tspan)); % mode

% Physical Parameters
params.fibula       = 0.4;
params.femur        = 0.4;
params.tarsal       = 0.05;              
params.HipWidth     = 0.2;
params.StepSize     = 0.25;
params.mode         = 0;    % BOTH  FIXED
%                    -1;    % LEFT  FIXED - FKM               
%                     1;    % RIGHT FIXED - FKM
% Masses
params.mass.femur   = 1;    % Thigh Bone
params.mass.fibula  = 1;    % Paired with `tibia`
params.mass.joint   = 0.5;  % Knee Bone / Joints / Ankles
params.mass.pelvis  = 0.7;  % Waist
params.mass.foot    = 0.1;  % Foot

%% PROBLEM:
%  - Stepping with LEFT
%   - FKM LEFT to RIGHT
%   - Requires: LEFT in BODY co-ords
%               CoM in BODY co-ords
%  - Shifting MASS
%   - FKM LEFT & RIGHT to BODY
%   - Requires: LEFT & RIGHT in BODY co-ords
%               CoM in BODY co-ords

%% Initial Position & Orientation & Conditions & Figure
    model.q0 = [pi/6;      % θ₁
             -2*pi/6;      % θ₂
                pi/6;      % θ₃
                pi/6;      % θ₄
             -2*pi/6;      % θ₅
                pi/6];     % θ₆
    
    model.rGBg(:,1) = [ 0; 0; 0];
    model.q(:,1)    = model.q0;
   [model.rBLb(:,1), ...
    model.rBRb(:,1), ...
    HTs]            = k_init(model.q0, params);

INTITAL_FIGURE = figure(1);
    clf(INTITAL_FIGURE)
    hold on
    grid on
    view(55,35);
    title("2D Model - LOCAL");
    set(gca, 'zdir','reverse');
    set(gca, 'ydir','reverse');
    set(gca,'Color','#EEEEEE');
    xlabel('{\bfX} (metres)');
    ylabel('{\bfY} (metres)');
    zlabel('{\bfZ} (metres)');
    
    % ZERO: X,Y,Z    X, Y, Z
    quiver3(0,0,0, 0.5, 0, 0,'r','LineWidth',2); % X
    quiver3(0,0,0, 0, 0.5, 0,'b','LineWidth',2); % Y
    quiver3(0,0,0, 0, 0, 0.5,'g','LineWidth',2); % Z

    % MAIN COMPONENTS
    plot3(model.rGBg(1),model.rGBg(2),model.rGBg(3), ...    % BODY
          'mx','LineWidth',2,'MarkerSize',10);
    
    plot3(model.xe(1,1),model.xe(2,1),model.xe(3,1), ...    % END EFFECTOR
          'ko','LineWidth',2,'MarkerSize',10);
    % LEFT
    rG0_L = HTs.AG0_L(1:3,4);
    plot3(rG0_L(1),rG0_L(2),rG0_L(3),'bx','LineWidth',2,'MarkerSize',5)
    plot3([model.rGBg(1) rG0_L(1)], ...
          [model.rGBg(2) rG0_L(2)], ...
          [model.rGBg(3) rG0_L(3)],'k', 'LineWidth',2);
    rG1_L = HTs.AG1_L(1:3,4);
    plot3(rG1_L(1),rG1_L(2),rG1_L(3),'bx','LineWidth',2,'MarkerSize',10)
    plot3([rG0_L(1) rG1_L(1)],[rG0_L(2) rG1_L(2)],[rG0_L(3) rG1_L(3)], ...
           'k', 'LineWidth',2);
    rG2_L = HTs.AG2_L(1:3,4);
    plot3(rG2_L(1),rG2_L(2),rG2_L(3),'bx','LineWidth',2,'MarkerSize',10)
    plot3([rG1_L(1) rG2_L(1)],[rG1_L(2) rG2_L(2)],[rG1_L(3) rG2_L(3)], ...
           'k', 'LineWidth',2);
    rG3_L = HTs.AG3_L(1:3,4);
    plot3(rG3_L(1),rG3_L(2),rG3_L(3),'bx','LineWidth',2,'MarkerSize',10)
    plot3([rG2_L(1) rG3_L(1)],[rG2_L(2) rG3_L(2)],[rG2_L(3) rG3_L(3)], ...
           'k', 'LineWidth',2);
    % RIGHT
    rG0_R = HTs.AG0_R(1:3,4);
    plot3(rG0_R(1),rG0_R(2),rG0_R(3),'bx','LineWidth',2,'MarkerSize',5)
    plot3([model.rGBg(1) rG0_R(1)], ...
          [model.rGBg(2) rG0_R(2)], ...
          [model.rGBg(3) rG0_R(3)],'k', 'LineWidth',2);
    rG1_R = HTs.AG1_R(1:3,4);
    plot3(rG1_R(1),rG1_R(2),rG1_R(3),'rx','LineWidth',2,'MarkerSize',10)
    plot3([rG0_R(1) rG1_R(1)],[rG0_R(2) rG1_R(2)],[rG0_R(3) rG1_R(3)], ...
           'k', 'LineWidth',2);
    rG2_R = HTs.AG2_R(1:3,4);
    plot3(rG2_R(1),rG2_R(2),rG2_R(3),'rx','LineWidth',2,'MarkerSize',10)
    plot3([rG1_R(1) rG2_R(1)],[rG1_R(2) rG2_R(2)],[rG1_R(3) rG2_R(3)], ...
           'k', 'LineWidth',2);
    rG3_R = HTs.AG3_R(1:3,4);
    plot3(rG3_R(1),rG3_R(2),rG3_R(3),'rx','LineWidth',2,'MarkerSize',10)
    plot3([rG2_R(1) rG3_R(1)],[rG2_R(2) rG3_R(2)],[rG2_R(3) rG3_R(3)], ...
           'k', 'LineWidth',2);

    legend({'+X_0','+Y_0','+Z_0','{\bfr}_B^{global}'},...
            Location='west');
    
    %    [MIN,   MAX,
    axis([ -1,     1, ... % X
           -0.5, 0.5, ... % Y
           -0.05,  1]);   % Z

%% MAIN LOOPs
tic % START TIMING
% Trajectory Generation
[Q1,~,~] = trajectoryGeneration(1, model, 1:61, params);
 Q = Q1;

for i=2:61
    model.xe(:,i)   = [Q(:,i); zeros(3,1)];
    model.q(:,i)    = k_Inv(model.q(:,i-1), model.xe(:,i), i, model, params);
    [~, HTs]        = k(model.q(:,i),(i-1),model, params);
    model.rBLb(:,i) = HTs.ABLb(1:3,4);
    model.rBRb(:,i) = HTs.ABRb(1:3,4);
    model.rBHb(:,i) = HTs.AbH(1:3,4);
    model.rCoMb(:,i) = rCoM(model.q(:,i),i,model,params);
    model.mode(:,i) = params.mode;
end

params.mode = 0;
% Trajectory Generation
[Q2,~,~] = trajectoryGeneration(61, model, 62:121, params);
 Q = [Q Q2];

for i=62:121
    model.xe(:,i)   = [Q(:,i); zeros(3,1)];
    model.q(:,i)    = k_Inv(model.q(:,i-1), model.xe(:,i), i, model, params);
    [~, HTs]        = k(model.q(:,i),(i-1),model, params);
    model.rBLb(:,i) = HTs.ABLb(1:3,4);
    model.rBRb(:,i) = HTs.ABRb(1:3,4);
    model.rBHb(:,i) = HTs.AbH(1:3,4);
    model.rCoMb(:,i) = rCoM(model.q(:,i),i,model,params);
    model.mode(:,i) = params.mode;
end


params.mode = 1;
% Trajectory Generation
[Q3,~,~] = trajectoryGeneration(121, model, 122:181, params);
 Q = [Q Q3];

for i=122:181
    model.xe(:,i)   = [Q(:,i); zeros(3,1)];
    model.q(:,i)    = k_Inv(model.q(:,i-1), model.xe(:,i), i, model, params);
    [~, HTs]     = k(model.q(:,i),(i-1),model, params);
    model.rBLb(:,i) = HTs.ABLb(1:3,4);
    model.rBRb(:,i) = HTs.ABRb(1:3,4);
    model.rBHb(:,i) = HTs.AbH(1:3,4);
    model.rCoMb(:,i) = rCoM(model.q(:,i),i,model,params);
    model.mode(:,i) = params.mode;
end

params.mode = 0;
% Trajectory Generation
[Q4,~,~] = trajectoryGeneration(181, model, 182:241, params);
 Q = [Q Q4];

for i=182:241
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
    [~, HTs] = kSLOW(model.q(:,i), i, model, params);

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
 
    % ZERO: Z,X,Y,   Z,  X,  Y
    quiver3(0,0,0, 0.5,0.0,0.0,'b','LineWidth',2); % Z
    quiver3(0,0,0, 0.0,0.5,0.0,'r','LineWidth',2); % X
    quiver3(0,0,0, 0.0,0.0,0.5,'g','LineWidth',2); % Y
    
    % MAIN COMPONENTS
    rBB = HTs.AbB(1:3,4);
    rB0 = HTs.Ab0(1:3,4);
    plot3([rBB(3), rB0(3)], ... Z
          [rBB(1), rB0(1)], ... X
          [rBB(2), rB0(2)], ... Y
        'k', 'LineWidth',2);
    plot3(rB0(3), rB0(1), rB0(2), 'rx', 'LineWidth',2,'MarkerSize',10);
    rB1 = HTs.Ab1(1:3,4);
    plot3([rB0(3) rB1(3)],[rB0(1) rB1(1)],[rB0(2) rB1(2)],...
        'k', 'LineWidth',2);
    plot3(rB1(3), rB1(1), rB1(2) ,'rx', 'LineWidth',2,'MarkerSize',10);
    rB2 = HTs.Ab2(1:3,4);
    plot3([rB1(3) rB2(3)], [rB1(1) rB2(1)], [rB1(2) rB2(2)],...
        'k', 'LineWidth',2);
    plot3(rB2(3), rB2(1), rB2(2) ,'rx', 'LineWidth',2,'MarkerSize',10);
    rbH = HTs.AbH(1:3,4);
    plot3(rbH(3), rbH(1), rbH(2), 'mx', 'LineWidth',2,'MarkerSize',10);
    rB3 = HTs.Ab3(1:3,4);
    plot3([rB2(3) rB3(3)], [rB2(1) rB3(1)], [rB2(2) rB3(2)],...
        'k', 'LineWidth',2);
    plot3(rB3(3), rB3(1), rB3(2), 'bx', 'LineWidth',2,'MarkerSize',10);
    rB4 = HTs.Ab4(1:3,4);
    plot3([rB3(3) rB4(3)], [rB3(1) rB4(1)], [rB3(2) rB4(2)],...
    'k', 'LineWidth',2);
    plot3(rB4(3), rB4(1), rB4(2), 'bx', 'LineWidth',2,'MarkerSize',10);
    rB5 = HTs.Ab5(1:3,4);
    plot3([rB4(3) rB5(3)], [rB4(1) rB5(1)], [rB4(2) rB5(2)],...
        'k', 'LineWidth',2);
    plot3(rB5(3), rB5(1), rB5(2), 'bx', 'LineWidth',2,'MarkerSize',10);
    rB6 = HTs.Ab6(1:3,4);
    plot3([rB5(3) rB6(3)], [rB5(1) rB6(1)], [rB5(2) rB6(2)],...
        'k', 'LineWidth',2);
    plot3(rB6(3), rB6(1), rB6(2), 'bx', 'LineWidth',2,'MarkerSize',10);
    rBE = HTs.AbE(1:3,4);
    plot3([rB6(3) rBE(3)], [rB6(1) rBE(1)], [rB6(2) rBE(2)],...
        'k', 'LineWidth',2);
    plot3(rBE(3), rBE(1), rBE(2), 'ko', 'LineWidth',2,'MarkerSize',10);

    plot3(model.rCoMb(3,i)*[1 1], ...
          model.rCoMb(1,i)*[1 1], ...
          model.rCoMb(2,i)*[1 0],'m:','LineWidth',2)

    legend({'+Z_0','+X_0','+Y_0','{r}^B_0 - \it{Link 0}',...
            'Joints (1 - 3)','','','','','Mid Waist','',...
            'Joints (4 - 6)','','','','','','','', ...
            'End Effector', 'CoM'},...
            Location='west');
    
    %    [      MIN,      MAX, ...
    axis([     -0.4,      0.2, ... % Z
           rbH(1)-1, rbH(1)+1, ... % X
                  0,        1]);   % Y
    
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
