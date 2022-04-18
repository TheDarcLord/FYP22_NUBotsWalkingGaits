clear all
close all
clc

tic % START TIMING

%% NOTES:
% 2D KINEMATICALLY REDUNDANT
%
% Rotation Parameterisation ZYX - Ψθϕ
%   ϕ = atan2( R₃₂, R₃₃)
%   θ = atan2(-R₃₁, SQRT(R₃₂² + R₃₃²))
%   Ψ = atan2( R₂₁, R₁₁)
% THIS PROBLEM:
% R₁₁:  cθ₁cθ₂
% R₂₁:  sθ₁cθ₂
% R₃₁: -sθ₂
% R₃₂:  0
% R₃₃:  cθ₂
    
%% Setup
params.framerate    = 20;
params.orientation  = 1;
model.tspan         = 0:(1 / params.framerate):6;
model.q             = zeros(6,length(model.tspan)); % Making Space
model.xe            = zeros(6,length(model.tspan)); % Making Space
params.fibula       = 0.5;
params.femur        = 0.5;
params.HipWidth     = 0.25;
H                   = params.HipWidth;
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

model.xe0 = [0; % X
             0; % Y
            -H; % Z
             0; % ϕ
             0; % θ
             0];% Ψ

%% Trajectory
D = diag(1:5,-1);   % Special D - Diag Matrix

q0 = [0.00  0.00  0.00;  % qX vX aX
      0.00  0.00  0.00;  % qY vY aY
     -H     0.00  0.00]; % qZ vZ aZ
t0 = 0;
tt0 = t0.^(0:5).';
T0 = [tt0, D*tt0, D^2*tt0];

q1 = [0.00  0.00  0.00;  % qX vX aX
      0.00  0.00  0.00;  % qY vY aY
     -H     0.00  0.00]; % qZ vZ aZ
t1 = 1;
tt1 = t1.^(0:5).';
T1 = [tt1, D*tt1, D^2*tt1];

q2 = [0.25  0.00  0.00;  % qX vX aX
      0.25  0.00  0.00;  % qY vY aY
     -H     0.00  0.00]; % qZ vZ aZ
t2 = 3;
tt2 = t2.^(0:5).';
T2 = [tt2, D*tt2, D^2*tt2];

q3 = [0.50  0.00  0.00;  % qX vX aX
      0.00  0.00  0.00;  % qY vY aY
     -H     0.00  0.00]; % qZ vZ aZ
t3 = 4;
tt3 = t3.^(0:5).';
T3 = [tt3, D*tt3, D^2*tt3];

q4 = [0.50  0.00  0.00;  % qX vX aX
      0.00  0.00  0.00;  % qY vY aY
     -H     0.00  0.00]; % qZ vZ aZ
t4 = 5;
tt4 = t4.^(0:5).';
T4 = [tt4, D*tt4, D^2*tt4];

q5 = [0.50  0.00  0.00;  % qX vX aX
      0.00  0.00  0.00;  % qY vY aY
     -H     0.00  0.00]; % qZ vZ aZ
t5 = 6;
tt5 = t5.^(0:5).';
T5 = [tt5, D*tt5, D^2*tt5];

C = [q0 q1 q2 q3 q4 q5]/[T0 T1 T2 T3 T4 T5];

% Evaluate postions and time derivatives
tt = model.tspan.^((0:5).');
Q = C*tt;    % Targeting Postions First ... Worry about Velocities later
V = C*D*tt;
A = C*D^2*tt;

%% LOOP

% T0 - * TO DEAL WITH FOR LOOP*
%model.xe(:,1) = model.xe0;
model.q(:,1) = model.q0;
[model.xe(:,1), ~, ~] = k(model.q0, params);

for i=2:length(model.tspan)
    model.xe(:,i) = [Q(:,i); zeros(3,1)];
    model.q(:,i) = k_Inv(model.q(:,i-1), model.xe(:,i), params);
end

toc % FINISH TIMING

%% Graph
figure('Name','Animation')

for i=1:length(model.tspan)
    
    [~, ~, HomegeneousTransforms] = k(model.q(:,i), params);
    r0CoM = rCoM(model.q(:,i),params);

    % DEBUG
    %[~, T16, HomegeneousTransforms] = k(model.q0, params);
    %r0CoM = rCoM(model.q0, params);

    clf
    hold on
    grid on
    title("Step One")
    txt = " Time: " + num2str(model.tspan(i)) + " sec";
    text(0,2,2,txt)
    
    % ZERO:  Z      X      Y
    plot3([0 1], [0 0], [0 0],'r', 'LineWidth',0.5); % Z
    plot3([0 0], [0 1], [0 0],'g', 'LineWidth',0.5); % X
    plot3([0 0], [0 0], [0 1],'b', 'LineWidth',0.5); % Y
    plot3(Q(3,1:i),Q(1,1:i),Q(2,1:i), 'k-','LineWidth', 0.5)
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
    
    % CoM
    plot3(r0CoM(3),r0CoM(1),0, 'rx', 'LineWidth',2);

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
