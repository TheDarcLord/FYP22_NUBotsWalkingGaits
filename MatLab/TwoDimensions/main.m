clear all
close all
clc

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
params.orientation  = 1;
model.tspan         = 0:0.1:5;
model.q             = zeros(6,length(model.tspan)); % Making Space
model.xe            = zeros(6,length(model.tspan)); % Making Space
params.fibula       = 0.5;
params.femur        = 0.5;
params.HipWidth     = 0.3;

% Masses
params.mass.femur   = 1;    % Thigh Bone
params.mass.fibula  = 1;    % Paired with `tibia`
params.mass.joint   = 0.5;  % Knee Bone / Joints
params.mass.pelvis  = 1.5;  % Waist 


%% Initial Position & Orientation

model.q0 = [-pi/15; % θ₁
             2*pi/15; % θ₂
            -pi/15;     % θ₃
             pi/15; % θ₄
            -2*pi/15; % θ₅
             0];    % θ₆

model.xe0 = [0; % X
             0; % Y
          -0.3; % Z
             0; % ϕ
             0; % θ
             0];% Ψ


%% Trajectory
D = diag(1:3,-1);   % Special D - Diag Matrix

q0 = [0 0;  % qX vX
      0 0;  % qY vY
   -0.3 0]; % qZ vZ
t0 = 1;
tt0 = t0.^(0:3).';
T0 = [tt0, D*tt0];

q1 = [0 0;  % qX vX
      0 0;  % qY vY
   -0.3 0]; % qZ vZ
t1 = 2;
tt1 = t1.^(0:3).';
T1 = [tt1, D*tt1];

q2 = [0.25   0;  % qX vX
      0.1    0;  % qY vY
     -0.3   0]; % qZ vZ
t2 = 3;
tt2 = t2.^(0:3).';
T2 = [tt2, D*tt2];

q3 = [0.5 0;  % qX vX
      0 0;   % qY vY
     -0.3  0]; % qZ vZ
t3 = 4;
tt3 = t3.^(0:3).';
T3 = [tt3, D*tt3];

C = [q0 q1 q2 q3]/[T0 T1 T2 T3];

% Evaluate postions and time derivatives
tt = model.tspan.^((0:3).');
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


%% Graph
figure('Name','Animation')
for i=1:length(model.tspan)
    
    [~, ~, HomegeneousTransforms] = k(model.q(:,i), params);
    r0CoM = rCoM(model.q(:,i),params);

    % DEBUG
    %[~, ~, HomegeneousTransforms] = k(model.q0, params);
    %r0CoM = rCoM(model.q0, params);

    clf
    hold on
    
    txt = {'Plotted Data:','y = sin(x)'};
    text(0,1.5,1.5,txt)
    % Z X Y
    % ZERO
    plot3([0 3], [0 0], [0 0],'r', 'LineWidth',0.5); % Z
    plot3([0 0], [0 3], [0 0],'g', 'LineWidth',0.5); % X
    plot3([0 0], [0 0], [0 3],'b', 'LineWidth',0.5); % Y
    legend('+Z','+X','+Y','Autoupdate','off');
    % ONE
    r01 = HomegeneousTransforms.A01(1:3,4);
    plot3(r01(3), r01(1), r01(2), 'gx', 'LineWidth',2);         % JOINT ONE
    % TWO
    r02 = HomegeneousTransforms.A02(1:3,4);
    plot3([r01(3) r02(3)], [r01(1) r02(1)], [r01(2) r02(2)],... % LINK
    'k', 'LineWidth',2);
    plot3(r02(3), r02(1), r02(2) ,'gx', 'LineWidth',2);         % JOINT TWO
    % THREE
    r03 = HomegeneousTransforms.A03(1:3,4);
    plot3([r02(3) r03(3)], [r02(1) r03(1)], [r02(2) r03(2)],... % LINK
    'k', 'LineWidth',2);
    plot3(r03(3), r03(1), r03(2), 'gx', 'LineWidth',2);         % JOINT THREE
    % FOUR
    r04 = HomegeneousTransforms.A04(1:3,4);
    plot3([r03(3) r04(3)], [r03(1) r04(1)], [r03(2) r04(2)],... % LINK
    'k', 'LineWidth',2);
    plot3(r04(3), r04(1), r04(2), 'bx', 'LineWidth',2);         % JOINT FOUR
    % FIVE
    r05 = HomegeneousTransforms.A05(1:3,4);
    plot3([r04(3) r05(3)], [r04(1) r05(1)], [r04(2) r05(2)],... % LINK
    'k', 'LineWidth',2);
    plot3(r05(3), r05(1), r05(2), 'bx', 'LineWidth',2);         % JOINT FIVE
    % SIX 
    r06 = HomegeneousTransforms.A06(1:3,4);
    plot3([r05(3) r06(3)], [r05(1) r06(1)], [r05(2) r06(2)],... % LINK
    'k', 'LineWidth',2);
    plot3(r06(3), r06(1), r06(2), 'bx', 'LineWidth',2);         % JOINT SIX
    
    % CoM
    plot3(r0CoM(3),r0CoM(1),0, 'rx', 'LineWidth',2);

    %        X    Y   Z
    %        Z    X   Y
    axis([-1 1 -0.5 2 0 2]);
    view(135,35);
    %view(90,0); % -> 2D
    pause(0.01)
end

