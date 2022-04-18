function [xe, T16, HTs] = k(q, params)
% k(q)  [2D Model] Forward Kinematic Model - FKM
%       
%       Returns:    [xe, T16, Transforms] for an array of 'q'
%       xe:         End Effector Pose [X Y Z ϕ θ Ψ]'
%       T16:        Transform from ONE to END EFFECTOR
%       Transforms: All other Homogenous Transforms

    %% LINK VARIABLES
    Ll     = params.fibula; % Lower Leg
    Lu     = params.femur;  % Upper Leg
    H      = params.HipWidth;

    %% JOINT VARIABLES
    sigT        = sum(q);        % Σθᵢ    i=1:6
    sigT15      = sum(q(1:5));   % Σθᵢ    i=1:5
    sigT14      = sum(q(1:4));   % Σθᵢ    i=1:4
    sigT13      = sum(q(1:3));   % Σθᵢ    i=1:3
    sigT12      = sum(q(1:2));   % Σθᵢ    i=1:2

    %% HOMOGENOUS TRANSFORM
    % LEFT FIXED
    T16x = Lu*(sin(sigT14) - sin(sigT12)) + ...
           Ll*(sin(sigT15) - sin( q(1) ));
    T16y = Lu*(cos(sigT12) - cos(sigT14)) + ...
           Ll*(cos( q(1) ) - cos(sigT15));
    T16z = -H;
   
    T16 = [cos(sigT), -sin(sigT), 0, T16x;
           sin(sigT),  cos(sigT), 0, T16y;
                   0,          0, 1, T16z;
                   0,          0, 0,    1];

    %% End Effector Parameterisation
    % R₁₁:  cθ₁cθ₂      Ψ = atan2( R₃₂, R₃₃)
    % R₂₁:  sθ₁cθ₂      θ = atan2(-R₃₁, SQRT(R₃₂² + R₃₃²))
    % R₃₁: -sθ₂         ϕ = atan2( R₂₁, R₁₁)
    % R₃₂:  0
    % R₃₃:  cθ₂

    phi =   atan2( T16(3,2), T16(3,3) );  
    theta = atan2(-T16(3,1), sqrt( T16(3,2)^2 + T16(3,3)^2 ) );
    psi =   atan2( T16(2,1), T16(1,1) );
    
    xe = [T16(1:3,4); % X Y Z
          phi;        % ϕ
          theta;      % θ
          psi];       % Ψ
    
    %% Homogeneous Transforms
    T     = @(x, y, z) [    eye(3), [x;y;z] ;
                        zeros(1,3),    1   ];

    A01 = T(0,0,0);
    HTs.A01 = A01;

    % A01 * A12 = A02
        A12x = -Ll*sin(q(1));
        A12y =  Ll*cos(q(1));
    HTs.A02  = A01*[cos( q(1) ), -sin( q(1) ), 0, A12x;
                    sin( q(1) ),  cos( q(1) ), 0, A12y;
                              0,            0, 1,    0;
                              0,            0, 0,    1];
    % A01 * A13 = A03
        A13x = -Lu*sin(sigT12)-Ll*sin(q(1));
        A13y =  Lu*cos(sigT12)+Ll*cos(q(1));
    HTs.A03  = A01*[cos(sigT12), -sin(sigT12), 0, A13x;
                    sin(sigT12),  cos(sigT12), 0, A13y;
                              0,            0, 1,    0;
                              0,            0, 0,    1];
    % A01 * A14 = A04
        A14x = -Lu*sin(sigT12)-Ll*sin(q(1));
        A14y =  Lu*cos(sigT12)+Ll*cos(q(1));
    HTs.A04  = A01*[cos(sigT13), -sin(sigT13), 0, A14x;
                    sin(sigT13),  cos(sigT13), 0, A14y;
                              0,            0, 1,   -H;
                              0,            0, 0,    1];
    % A01 * A15 = A05
        A15x =  Lu*(sin(sigT14)-sin(sigT12))-Ll*sin(q(1));
        A15y =  Lu*(cos(sigT12)-cos(sigT14))+Ll*cos(q(1));
    HTs.A05  = A01*[cos(sigT14), -sin(sigT14), 0,  A15x;
                    sin(sigT14),  cos(sigT14), 0,  A15y;
                              0,            0, 1,    -H;
                              0,            0, 0,     1];
    % A01 * T16 = A06
    HTs.A06 = A01*T16;

end