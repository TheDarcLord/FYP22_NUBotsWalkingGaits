clear all
close all
clc 

tic % START TIMING

%% Setup
params.framerate    = 10;
model.tspan         = 0:(1 / params.framerate):30;

model.q             = zeros(6,length(model.tspan)); % q   [θ₁θ₂θ₃θ₄θ₅θ₆]'
model.xe            = zeros(7,length(model.tspan)); % xe  [XYZϕθΨ]'
model.r01g          = zeros(3,length(model.tspan)); % A01 [XYZ]'
model.r06g          = zeros(3,length(model.tspan)); % A06 [XYZ]'
model.r0Hg          = zeros(3,length(model.tspan)); % A0H [XYZ]'   

% Physical Parameters
params.fibula       = 0.5;
params.femur        = 0.5;
params.HipWidth     = 0.25;
params.r0Ag         = zeros(3,1);  % Ankle Position from 0rigin in Global
params.step         = 1;           % OddStep:  LEFT FIXED
                                   % EvenStep: RIGHT FIXED
params.waistHeight  = 0.8830;
% Masses
params.mass.femur   = 1;    % Thigh Bone
params.mass.fibula  = 1;    % Paired with `tibia`
params.mass.joint   = 0.5;  % Knee Bone / Joints
params.mass.pelvis  = 1.5;  % Waist

%% Initial Position & Orientation

model.q0 = [-pi/6;             % θ₁
           2*pi/6;             % θ₂
            -pi/6;             % θ₃
             pi/6;             % θ₄
          -2*pi/6;             % θ₅
             pi/6];            % θ₆

model.xe0 = [0;                 % X
             0;                 % Y
            -params.HipWidth;   % Z
             0;                 % ϕ
             0;                 % θ
             0;                 % Ψ
             params.waistHeight]; % WaistHeight

%% LOOP
% Initial Conditions
model.q(:,1)                = model.q0;
[model.xe(:,1), ~, HTs]     = k(model.q0, params);
model.r06g(:,1)             = HTs.A06(1:3,4);
model.r01g(:,1)             = HTs.A01(1:3,4);
model.r0Hg(:,1)             = HTs.A0H(1:3,4);
params.r0Ag                 = model.r01g(:,1);
[Q,~,~] = trajectoryGeneration(model, params); % Trajectory Generation

for i=2:61
    model.xe(:,i)   = [Q(:,i); zeros(3,1); params.waistHeight];
    model.q(:,i)    = k_Inv(model.q(:,i-1), model.xe(:,i), params);
    [~, ~, HTs]     = k(model.q(:,i), params);
    model.r01g(:,i) = HTs.A01(1:3,4);
    model.r06g(:,i) = HTs.A06(1:3,4);
    model.r0Hg(:,i) = HTs.A0H(1:3,4);
end

params.r0Ag = model.r06g(:,61);
params.step = 2;
[Q,~,~] = trajectoryGeneration(model, params); % Trajectory Generation

for i=62:121
    model.xe(:,i)   = [Q(:,i-61); zeros(3,1); params.waistHeight];
    model.q(:,i)    = k_Inv(model.q(:,i-1), model.xe(:,i), params);
    [~, ~, HTs]     = k(model.q(:,i), params);
    model.r01g(:,i) = HTs.A01(1:3,4);
    model.r06g(:,i) = HTs.A06(1:3,4);
    model.r0Hg(:,i) = HTs.A0H(1:3,4);
end

params.r0Ag = model.r01g(:,121);
params.step = 3;
[Q,~,~] = trajectoryGeneration(model, params); % Trajectory Generation

for i=122:181
    model.xe(:,i)   = [Q(:,i-121); zeros(3,1); params.waistHeight];
    model.q(:,i)    = k_Inv(model.q(:,i-1), model.xe(:,i), params);
    [~, ~, HTs]     = k(model.q(:,i), params);
    model.r01g(:,i) = HTs.A01(1:3,4);
    model.r06g(:,i) = HTs.A06(1:3,4);
    model.r0Hg(:,i) = HTs.A0H(1:3,4);
end

params.r0Ag = model.r06g(:,181);
params.step = 4;
[Q,~,~] = trajectoryGeneration(model, params); % Trajectory Generation

for i=182:241
    model.xe(:,i)   = [Q(:,i-181); zeros(3,1); params.waistHeight];
    model.q(:,i)    = k_Inv(model.q(:,i-1), model.xe(:,i), params);
    [~, ~, HTs]     = k(model.q(:,i), params);
    model.r01g(:,i) = HTs.A01(1:3,4);
    model.r06g(:,i) = HTs.A06(1:3,4);
    model.r0Hg(:,i) = HTs.A0H(1:3,4);
end

params.r0Ag = model.r01g(:,241);
params.step = 5;
[Q,~,~] = trajectoryGeneration(model, params); % Trajectory Generation

for i=242:length(model.tspan)
    model.xe(:,i)   = [Q(:,i-241); zeros(3,1); params.waistHeight];
    model.q(:,i)    = k_Inv(model.q(:,i-1), model.xe(:,i), params);
    [~, ~, HTs]     = k(model.q(:,i), params);
    model.r01g(:,i) = HTs.A01(1:3,4);
    model.r06g(:,i) = HTs.A06(1:3,4);
    model.r0Hg(:,i) = HTs.A0H(1:3,4);
end

toc % FINISH TIMING

%% Graph/Figures 
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

figure('Name','Foot & Waist Movement')
    hold on
    grid on
    title('Foot Trajectories','FontSize',12);
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
    legend('+Z','+X','+Y','Left','Right','Waist');
    axis([-(params.HipWidth+0.2) 0.2 min(model.r0Hg(1,:))-0.1 max(model.r0Hg(1,:))+0.2 0 1]);
    view(135,35);
%%
figure('Name','Animation')
for i=1:length(model.tspan)

    clf
    hold on
    grid on
    
    txt = " Time: " + num2str(model.tspan(i)) + " sec";
    text(0,2,2,txt)

    if i > 241
        params.r0Ag = model.r01g(:,241);
        params.step = 5;
    elseif i > 181
        params.r0Ag = model.r06g(:,181);
        params.step = 4;
    elseif i > 121
        params.r0Ag = model.r01g(:,121);
        params.step = 3;
    elseif i > 61
        params.r0Ag = model.r06g(:,61);
        params.step = 2;
    else 
        params.r0Ag = model.r01g(:,1);
        params.step = 1;
    end
    set(gca,'Color','#CCCCCC');
    [~, ~, HomegeneousTransforms] = k(model.q(:,i), params);
    r0CoM = rCoM(model.q(:,i),params);

    % DEBUG
    %[xe, ~, HomegeneousTransforms] = k(model.qTEST, params);

    % ZERO:  Z      X      Y
    plot3([0 1], [0 0], [0 0],'r', 'LineWidth',0.5); % Z
    plot3([0 0], [0 1], [0 0],'g', 'LineWidth',0.5); % X
    plot3([0 0], [0 0], [0 1],'b', 'LineWidth',0.5); % Y
    %plot3(Q(3,j:i),Q(1,j:i)+params.r0Ag(1),Q(2,j:i), 'k-','LineWidth', 0.5)
    legend('+Z','+X','+Y', 'Trajectory','Autoupdate','off');

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

    title("Stand | Initial Step | Multiple Steps")
    xlabel('Z','FontWeight','bold');
    ylabel('X','FontWeight','bold');
    zlabel('Y','FontWeight','bold');

    % CoM
    plot3(r0CoM(3),r0CoM(1),0, 'rx', 'LineWidth',1.5);

    %        X                         Y   Z
    %        Z                         X   Y
    axis([-1 1 (r0H(1))-1.1 (r0H(1)+1.1) 0 2]);
    view(135,35);
    %view(90,0); % -> 2D
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
