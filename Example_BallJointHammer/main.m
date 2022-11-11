clear all
close all
clc

%% NOTES:
% NOT KINEMATICALLY REDUNDANT
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
params.orientation  = 0;
model.tspan        = 0:0.1:2;
model.q            = zeros(4,length(model.tspan));
model.xe           = zeros(6,length(model.tspan));
%params.q(:,1)       = [];
%params.xe(:,1)      = [];
%% Initial Position & Orientation

q0 = [-pi/4; % θ₁
      -pi/4; % θ₂
      1; % d₂
      1];% d₃

xe0 = [1; % X
      1; % Y
      1; % Z
      0; % ϕ
      0; % θ
      0];% Ψ

%% Trajectory
D = diag(1:3,-1);   % Special D - Diag Matrix

q0 = [0 0;  % qX vX
      0 0;  % qY vY
      0 0]; % qZ vZ
t0 = 0;
tt0 = t0.^(0:3).';
T0 = [tt0, D*tt0];

q1 = [1 0;  % qX vX
      0 0;  % qY vY
      0 0]; % qZ vZ
t1 = 1;
tt1 = t1.^(0:3).';
T1 = [tt1, D*tt1];

q2 = [1 0;  % qX vX
      0 0;  % qY vY
      1 0]; % qZ vZ
t2 = 2;
tt2 = t2.^(0:3).';
T2 = [tt2, D*tt2];

C = [q0 q1 q2]/[T0 T1 T2];

% Evaluate postions and time derivatives
tt = model.tspan.^((0:3).');
Q = C*tt;    % Targeting Postions First ... Worry about Velocities later
V = C*D*tt;
A = C*D^2*tt;

%% LOOP

% T0 - * TO DEAL WITH FOR LOOP*
xe = [Q(:,1); zeros(3,1)];
model.q(:,1) = k_Inv(zeros(4,1), xe, params);

for i=2:length(model.tspan)
    xe = [Q(:,i); zeros(3,1)];
    model.q(:,i) = k_Inv(model.q(:,i-1), xe, params);
end


%% Graph
figure('Name','Animation')
for i=1:length(model.tspan)
    
    [xe, T03, Transforms] = k(model.q(:,i), params);

    clf
    hold on
    %   ZERO
    plot3([0 3], [0 0], [0 0],'r', 'LineWidth',0.5);
    plot3([0 0], [0 3], [0 0],'g', 'LineWidth',0.5);
    plot3([0 0], [0 0], [0 3],'b', 'LineWidth',0.5);
    legend('+X','+Y','+Z','Autoupdate','off');
    %   ONE
    r02 = Transforms.A02(1:3,4);
    plot3(r02(1), r02(2), r02(3),'gx', 'LineWidth',2);
    plot3([0 r02(1)], [0 r02(2)], [0 r02(3)],'k', 'LineWidth',1);
    %   TWO
    r03 = T03(1:3,4);
    plot3(r03(1), r03(2), r03(3),'bx', 'LineWidth',2);
    plot3([r02(1) r03(1)], [r02(2) r03(2)], [r02(3) r03(3)],'k', 'LineWidth',1);
    
    axis([-3 3 -3 3 0 3]);
    view(105,35);
    pause(0.1)
end

