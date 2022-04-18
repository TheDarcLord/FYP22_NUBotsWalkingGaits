function [xe, T61, HTs] = k(q, params)
% k(q)  [2D Model] Forward Kinematic Model - FKM
%       
%       Returns:    [xe, T61, Transforms] for an array of 'q'
%       xe:         End Effector Pose [X Y Z ϕ θ Ψ]'
%       T61:        Transform from ONE to END EFFECTOR
%       Transforms: All other Homogenous Transforms

    %% LINK VARIABLES
    Ll     = params.fibula; % Lower Leg
    Lu     = params.femur;  % Upper Leg
    H      = params.HipWidth;

    %% JOINT VARIABLES
    sigT        = sum(q);        % Σθᵢ    i=1:6
    sigT12      = sum(q(1:2));   % Σθᵢ    i=1:2
    sigT13      = sum(q(1:3));   % Σθᵢ    i=1:3
    sigT14      = sum(q(1:4));   % Σθᵢ    i=1:4
    sigT26      = sum(q(2:6));   % Σθᵢ    i=2:6
    sigT36      = sum(q(3:6));   % Σθᵢ    i=3:6
    sigT46      = sum(q(4:6));   % Σθᵢ    i=4:6
    sigT56      = sum(q(5:6));   % Σθᵢ    i=5:6

    %% HOMOGENOUS TRANSFORM
    % RIGHT FIXED
    T61x = Lu*(sin(sigT56) - sin(sigT36)) + Ll*(sin(q(6)) - sin(sigT26));
    T61y = Lu*(cos(sigT56) - cos(sigT36)) + Ll*(cos(q(6)) - cos(sigT26));
    T61z = H;
    T61  = [cos(sigT),  sin(sigT), 0, T61x;
           -sin(sigT),  cos(sigT), 0, T61y;
                    0,          0, 1, T61z;
                    0,          0, 0,    1];

    %% End Effector Parameterisation
    % R₁₁:  cθ₁cθ₂      Ψ = atan2( R₃₂, R₃₃)
    % R₂₁:  sθ₁cθ₂      θ = atan2(-R₃₁, SQRT(R₃₂² + R₃₃²))
    % R₃₁: -sθ₂         ϕ = atan2( R₂₁, R₁₁)
    % R₃₂:  0
    % R₃₃:  cθ₂

    phi =   atan2( T61(3,2), T61(3,3) );  
    theta = atan2(-T61(3,1), sqrt( T61(3,2)^2 + T61(3,3)^2 ) );
    psi =   atan2( T61(2,1), T61(1,1) );
    
    xe = [T61(1:3,4); % X Y Z
          phi;        % ϕ
          theta;      % θ
          psi];       % Ψ
    
    %% Homogeneous Transforms
    T     = @(x, y, z) [    eye(3), [x;y;z] ;
                        zeros(1,3),    1   ];
    
    A06 = T(0,0,-0.25);

    % A06 * T61 = A01
    HTs.A01  = A06*T61;
        A62x = Lu*(sin(sigT56) - sin(sigT36)) + Ll*sin(q(6));
        A62y = Lu*(cos(sigT56) - cos(sigT36)) + Ll*cos(q(6));
    HTs.A02  = A06*[cos(sigT36), sin(sigT36), 0, A62x;
                   -sin(sigT36), cos(sigT36), 0, A62y;
                              0,           0, 1,    H;
                              0,           0, 0,    1];
    % A06 * A63 = A03
        A63x = Lu*sin(sigT56) + Ll*sin(q(6));
        A63y = Lu*cos(sigT56) + Ll*cos(q(6));
    HTs.A03  = A06*[cos(sigT46), sin(sigT46), 0, A63x;
                   -sin(sigT46), cos(sigT46), 0, A63y;
                              0,           0, 1,    H;
                              0,           0, 0,    1];
    % A06 * A64 = A04
        A64x = Lu*sin(sigT56) + Ll*sin(q(6));
        A64y = Lu*cos(sigT56) + Ll*cos(q(6));
    HTs.A04  = A06*[cos(sigT56), sin(sigT56), 0, A64x;
                   -sin(sigT56), cos(sigT56), 0, A64y;
                              0,           0, 1,    0;
                              0,           0, 0,    1];
        A65x = Ll*sin(q(6));
        A65y = Ll*cos(q(6));
    HTs.A05  = A06*[cos( q(6) ), sin( q(6) ), 0, A65x;
                   -sin( q(6) ), cos( q(6) ), 0, A65y;
                              0,           0, 1,    0;
                              0,           0, 0,    1];
    HTs.A06 = A06;
    
end