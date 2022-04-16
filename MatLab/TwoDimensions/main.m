clear all
close all
clc

%% NOTES:
% 2D KINEMATICALLY REDUNDANT ?? 
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
model.tspan         = 0:0.1:2;
model.q             = zeros(6,length(model.tspan)); % Making Space
model.xe            = zeros(6,length(model.tspan)); % Making Space
params.L1           = 0.5;
params.L2           = 0.5;
params.HipWidth     = 0.3;

%% Initial Position & Orientation

q0 = [0; % θ₁
      0; % θ₂
      0; % θ₃
      0; % θ₄
      0; % θ₅
      0];% θ₆
model.q(:,1) = q0;

xe0 = [0.5; % X
       0; % Y
    -0.3; % Z
       0; % ϕ
       0; % θ
       0];% Ψ
model.xe(:,1) = xe0;


%% Trajectory
% D = diag(1:3,-1);   % Special D - Diag Matrix
% 
% q0 = [0 0;  % qX vX
%       0 0;  % qY vY
%       0 0]; % qZ vZ
% t0 = 0;
% tt0 = t0.^(0:3).';
% T0 = [tt0, D*tt0];
% 
% q1 = [1 0;  % qX vX
%       0 0;  % qY vY
%       0 0]; % qZ vZ
% t1 = 1;
% tt1 = t1.^(0:3).';
% T1 = [tt1, D*tt1];
% 
% q2 = [1 0;  % qX vX
%       0 0;  % qY vY
%       1 0]; % qZ vZ
% t2 = 2;
% tt2 = t2.^(0:3).';
% T2 = [tt2, D*tt2];
% 
% C = [q0 q1 q2]/[T0 T1 T2];
% 
% % Evaluate postions and time derivatives
% tt = model.tspan.^((0:3).');
% Q = C*tt;    % Targeting Postions First ... Worry about Velocities later
% V = C*D*tt;
% A = C*D^2*tt;

% %% LOOP
% 
% % T0 - * TO DEAL WITH FOR LOOP*
% xe = [Q(:,1); zeros(3,1)];
% model.q(:,1) = k_Inv(zeros(4,1), xe, params);
% 
% for i=2:length(model.tspan)
%     xe = [Q(:,i); zeros(3,1)];
%     model.q(:,i) = k_Inv(model.q(:,i-1), xe, params);
% end


%% Graph
figure('Name','Animation')
% for i=1:length(model.tspan)
    
    %[xe, T03, Transforms] = k(model.q(:,i), params);
    % [xe, T16, Transforms] = k(q0, params);
    qStar = k_Inv(q0, xe0, params);
    [xe, T16, Transforms] = k(qStar, params);

    clf
    hold on
    % Z X Y
    %   ZERO
    plot3([0 3], [0 0], [0 0],'r', 'LineWidth',0.5); % Z
    plot3([0 0], [0 3], [0 0],'g', 'LineWidth',0.5); % X
    plot3([0 0], [0 0], [0 3],'b', 'LineWidth',0.5); % Y
    legend('+Z','+X','+Y','Autoupdate','off');
    % ONE
    r01 = [0;0;0];
    plot3(r01(3), r01(1), r01(2), 'gx', 'LineWidth',2);         % JOINT ONE
    % TWO
    r12 = Transforms.A12(1:3,4);
    plot3([r01(3) r12(3)], [r01(1) r12(1)], [r01(2) r12(2)],... % LINK
    'k', 'LineWidth',2);
    plot3(r12(3), r12(1), r12(2) ,'gx', 'LineWidth',2);         % JOINT TWO
    % THREE
    r13 = Transforms.A13(1:3,4);
    plot3([r12(3) r13(3)], [r12(1) r13(1)], [r12(2) r13(2)],... % LINK
    'k', 'LineWidth',2);
    plot3(r13(3), r13(1), r13(2), 'gx', 'LineWidth',2);         % JOINT THREE
    % FOUR
    r14 = Transforms.A14(1:3,4);
    plot3([r13(3) r14(3)], [r13(1) r14(1)], [r13(2) r14(2)],... % LINK
    'k', 'LineWidth',2);
    plot3(r14(3), r14(1), r14(2), 'bx', 'LineWidth',2);         % JOINT FOUR
    % FIVE
    r15 = Transforms.A15(1:3,4);
    plot3([r14(3) r15(3)], [r14(1) r15(1)], [r14(2) r15(2)],... % LINK
    'k', 'LineWidth',2);
    plot3(r15(3), r15(1), r15(2), 'bx', 'LineWidth',2);         % JOINT FIVE
    % SIX 
    r16 = Transforms.A16(1:3,4);
    plot3([r15(3) r16(3)], [r15(1) r16(1)], [r15(2) r16(2)],... % LINK
    'k', 'LineWidth',2);
    plot3(r16(3), r16(1), r16(2), 'bx', 'LineWidth',2);         % JOINT SIX
    

    %plot3([0 r12(1)], [0 r12(2)], [0 r12(3)],'k', 'LineWidth',1);
    
    %        X    Y   Z
    %        Z    X   Y
    axis([-1 1 -0.5 2 0 2]);
    view(125,35);
    %pause(0.1)
% end

