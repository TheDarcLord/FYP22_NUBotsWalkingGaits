clear all
close all
clc

tic % START TIMING

%% Setup
params.framerate    = 50;
model.tspan         = 0:(1 / params.framerate):12;

params.spanLow      = ceil(length(model.tspan)/2);
params.spanHigh     = ceil(length(model.tspan)/2)+1;

model.q             = zeros(6,length(model.tspan)); % Making Space
model.xe            = zeros(6,length(model.tspan)); % Making Space
params.fibula       = 0.5;
params.femur        = 0.5;
params.HipWidth     = 0.1;
H                   = params.HipWidth;
params.r0Ag         = zeros(3,1);  % Ankle Position from 0rigin in Global
params.step         = 1;           % OddStep:  LEFT FIXED
                                   % EvenStep: RIGHT FIXED
% Masses
params.mass.femur   = 1;    % Thigh Bone
params.mass.fibula  = 1;    % Paired with `tibia`
params.mass.joint   = 0.5;  % Knee Bone / Joints
params.mass.pelvis  = 1.5;  % Waist 

%% Initial Position & Orientation

model.q0 = [-pi/10; % θ₁
           2*pi/10; % θ₂
            -pi/10; % θ₃
             pi/10; % θ₄
          -2*pi/10; % θ₅
             pi/10];% θ₆

model.qTEST = [-0.5366
                0.4927
                0.0054
                0.6093
               -0.7262
                0.1554];

model.xe0 = [0; % X
             0; % Y
            -H; % Z
             0; % ϕ
             0; % θ
             0];% Ψ

model.xeTEST = [ 0.4704;
                 0.0145;
                -0.2500;
                      0;
                      0;
                      0];

%% Trajectory
[Q, V, A] = trajectoryGeneration(model, params);

% DEBUG
figure('Name','Trajectory')
    hold on
    grid on
    plot3([0 1], [0 0], [0 0],'r', 'LineWidth',0.5); % Z
    plot3([0 0], [0 1], [0 0],'g', 'LineWidth',0.5); % X
    plot3([0 0], [0 0], [0 1],'b', 'LineWidth',0.5); % Y
    plot3(Q(3,1:params.spanLow-1),...
          Q(1,1:params.spanLow-1),...
          Q(2,1:params.spanLow-1),...
        'k-','LineWidth', 2);
    plot3(Q(3,params.spanHigh:end),...
          Q(1,params.spanHigh:end)+0.5,...
          Q(2,params.spanHigh:end),...
        'k-','LineWidth', 2);
    legend('+Z','+X','+Y', 'Trajectories','Autoupdate','off');
    title('Trajectory Debug')
    axis([-1 1 -0.5 2 0 0.5]);
    xlabel('Z','FontWeight','bold');
    ylabel('X','FontWeight','bold');
    zlabel('Y','FontWeight','bold');
    view(135,35);

%% LOOP

% T0 - * TO DEAL WITH FOR LOOP*
%model.xe(:,1) = model.xe0;
model.q(:,1) = model.q0;
[model.xe(:,1), ~, ~] = k(model.q0, params);

for i=2:params.spanLow
    model.xe(:,i) = [Q(:,i); zeros(3,1)];
    model.q(:,i) = k_Inv(model.q(:,i-1), model.xe(:,i), params);
end

    params.r0Ag = model.xe(1:3,params.spanLow);
    params.step = 2;

for i=params.spanHigh:length(model.tspan)
    model.xe(:,i) = [Q(:,i); zeros(3,1)];
    model.q(:,i) = k_Inv(model.q(:,i-1), model.xe(:,i), params);
end


toc % FINISH TIMING

%% Graph
figure('Name','Animation')

for i=1:length(model.tspan)

    clf
    hold on
    grid on
    
    txt = " Time: " + num2str(model.tspan(i)) + " sec";
    text(0,2,2,txt)


    if i > params.spanLow 
        params.r0Ag = model.xe(1:3,params.spanLow);
        params.step = 2;
        j=params.spanHigh;
    else 
        params.r0Ag = zeros(3,1);
        params.step = 1;
        j=1;
    end

    [~, ~, HomegeneousTransforms] = k(model.q(:,i), params);
    r0CoM = rCoM(model.q(:,i),params);

    % DEBUG
    %[xe, ~, HomegeneousTransforms] = k(model.qTEST, params);

    % ZERO:  Z      X      Y
    plot3([0 1], [0 0], [0 0],'r', 'LineWidth',0.5); % Z
    plot3([0 0], [0 1], [0 0],'g', 'LineWidth',0.5); % X
    plot3([0 0], [0 0], [0 1],'b', 'LineWidth',0.5); % Y
    plot3(Q(3,j:i),Q(1,j:i)+params.r0Ag(1),Q(2,j:i), 'k-','LineWidth', 0.5)
    legend('+Z','+X','+Y', 'Trajectory','Autoupdate','off');


    % ONE
    r01 = HomegeneousTransforms.A01(1:3,4);
    plot3(r01(3), r01(1), r01(2), 'gx', 'LineWidth',2);         % J ONE
    % TWO
    r02 = HomegeneousTransforms.A02(1:3,4);
    plot3([r01(3) r02(3)], [r01(1) r02(1)], [r01(2) r02(2)],... % L TWO
    'k', 'LineWidth',2);
    plot3(r02(3), r02(1), r02(2) ,'gx', 'LineWidth',2);         % J TWO
    % THREE
    r03 = HomegeneousTransforms.A03(1:3,4);
    plot3([r02(3) r03(3)], [r02(1) r03(1)], [r02(2) r03(2)],... % L THREE
    'k', 'LineWidth',2);
    plot3(r03(3), r03(1), r03(2), 'gx', 'LineWidth',2);         % J THREE
    % FOUR
    r04 = HomegeneousTransforms.A04(1:3,4);
    plot3([r03(3) r04(3)], [r03(1) r04(1)], [r03(2) r04(2)],... % L FOUR
    'k', 'LineWidth',2);
    plot3(r04(3), r04(1), r04(2), 'bx', 'LineWidth',2);         % J FOUR
    % FIVE
    r05 = HomegeneousTransforms.A05(1:3,4);
    plot3([r04(3) r05(3)], [r04(1) r05(1)], [r04(2) r05(2)],... % L FIVE
    'k', 'LineWidth',2);
    plot3(r05(3), r05(1), r05(2), 'bx', 'LineWidth',2);         % J FIVE
    % SIX 
    r06 = HomegeneousTransforms.A06(1:3,4);
    plot3([r05(3) r06(3)], [r05(1) r06(1)], [r05(2) r06(2)],... % L SIX
    'k', 'LineWidth',2);
    plot3(r06(3), r06(1), r06(2), 'bx', 'LineWidth',2);         % J SIX

    title("Stand | Initial Step | 2ND Step")
    xlabel('Z','FontWeight','bold');
    ylabel('X','FontWeight','bold');
    zlabel('Y','FontWeight','bold');

    % CoM
    plot3(r0CoM(3),r0CoM(1),0, 'rx', 'LineWidth',1.5);

    %        X    Y   Z
    %        Z    X   Y
    axis([-1 1 -0.5 2 0 2]);
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
