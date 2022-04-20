function [xe, TAE, HTs] = k(q, params)
% k(q)  [2D Model] Forward Kinematic Model - FKM
%       
%       Returns:    [xe, TAA, Transforms] for an array of 'q'
%       xe:         End Effector Pose [X Y Z ϕ θ Ψ]'
%       TAA:        Transform from ANKLE to END EFFECTOR
%       Transforms: All other Homogenous Transforms

    %% LINK VARIABLES
    Ll     = params.fibula;     % Lower Leg
    Lu     = params.femur;      % Upper Leg
    H      = params.HipWidth;
    
    STEP   = params.step;           % Number of Steps
    A0A    = [eye(3), params.r0Ag;  % Ankle Position from 
              zeros(1,3),       1]; %  0rigin in Global

    %% JOINT VARIABLES
    sigT        = sum(q);        % Σθᵢ    i=1:6
    sigT15      = sum(q(1:5));   % Σθᵢ    i=1:5
    sigT14      = sum(q(1:4));   % Σθᵢ    i=1:4
    sigT13      = sum(q(1:3));   % Σθᵢ    i=1:3
    sigT12      = sum(q(1:2));   % Σθᵢ    i=1:2

    sigT26      = sum(q(2:6));   % Σθᵢ    i=2:6
    sigT36      = sum(q(3:6));   % Σθᵢ    i=3:6
    sigT46      = sum(q(4:6));   % Σθᵢ    i=4:6
    sigT56      = sum(q(5:6));   % Σθᵢ    i=5:6

    %% HOMOGENOUS TRANSFORM
    if rem(STEP,2) > 0
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
        TAE = T16;
    elseif rem(STEP,2) == 0
        % RIGHT FIXED
        T61x = Lu*(sin(sigT56) - sin(sigT36)) + ...
               Ll*(sin( q(6) ) - sin(sigT26));
        T61y = Lu*(cos(sigT56) - cos(sigT36)) + ... 
               Ll*(cos( q(6) ) - cos(sigT26));
        T61z = H;
        T61  = [cos(sigT),  sin(sigT), 0, T61x;
               -sin(sigT),  cos(sigT), 0, T61y;
                        0,          0, 1, T61z;
                        0,          0, 0,    1];
        TAE = T61;
    end

    %% End Effector Parameterisation
    % R₁₁:  cθ₁cθ₂      Ψ = atan2( R₃₂, R₃₃)
    % R₂₁:  sθ₁cθ₂      θ = atan2(-R₃₁, SQRT(R₃₂² + R₃₃²))
    % R₃₁: -sθ₂         ϕ = atan2( R₂₁, R₁₁)
    % R₃₂:  0
    % R₃₃:  cθ₂

    phi =   atan2( TAE(3,2), TAE(3,3) );  
    theta = atan2(-TAE(3,1), sqrt( TAE(3,2)^2 + TAE(3,3)^2 ) );
    psi =   atan2( TAE(2,1), TAE(1,1) );
    
    xe = [TAE(1:3,4); % X Y Z
          phi;        % ϕ
          theta;      % θ
          psi];       % Ψ
    
    %% Homogeneous Transforms
    
    if rem(STEP,2) > 0
        HTs.A01 = A0A;
        % A0A * A12 = A02
            A12x = -Ll*sin(q(1));
            A12y =  Ll*cos(q(1));
        HTs.A02  = A0A*[cos( q(1) ), -sin( q(1) ), 0, A12x;
                        sin( q(1) ),  cos( q(1) ), 0, A12y;
                                  0,            0, 1,    0;
                                  0,            0, 0,    1];
        % A0A * A13 = A03
            A13x = -Lu*sin(sigT12)-Ll*sin(q(1));
            A13y =  Lu*cos(sigT12)+Ll*cos(q(1));
        HTs.A03  = A0A*[cos(sigT12), -sin(sigT12), 0, A13x;
                        sin(sigT12),  cos(sigT12), 0, A13y;
                                  0,            0, 1,    0;
                                  0,            0, 0,    1];
        % A0A * A1H = A0H
            A1Hx = -Lu*sin(sigT12)-Ll*sin(q(1));
            A1Hy =  Lu*cos(sigT12)+Ll*cos(q(1));
        HTs.A0H  = A0A*[cos(sigT12), -sin(sigT12), 0, A1Hx;
                        sin(sigT12),  cos(sigT12), 0, A1Hy;
                                  0,            0, 1,  -H/2;
                                  0,            0, 0,    1];
        % A0A * A14 = A04
            A14x = -Lu*sin(sigT12)-Ll*sin(q(1));
            A14y =  Lu*cos(sigT12)+Ll*cos(q(1));
        HTs.A04  = A0A*[cos(sigT13), -sin(sigT13), 0, A14x;
                        sin(sigT13),  cos(sigT13), 0, A14y;
                                  0,            0, 1,   -H;
                                  0,            0, 0,    1];
        % A0A * A15 = A05
            A15x =  Lu*(sin(sigT14)-sin(sigT12))-Ll*sin(q(1));
            A15y =  Lu*(cos(sigT12)-cos(sigT14))+Ll*cos(q(1));
        HTs.A05  = A0A*[cos(sigT14), -sin(sigT14), 0,  A15x;
                        sin(sigT14),  cos(sigT14), 0,  A15y;
                                  0,            0, 1,    -H;
                                  0,            0, 0,     1];
        % A0A * T16 = A06
        HTs.A06 = A0A*T16;
    elseif rem(STEP,2) == 0
        % A06 * T61 = A01
        HTs.A01  = A0A*T61;
        % A06 * T62 = A02
            A62x = Lu*(sin(sigT56)-sin(sigT36))+Ll*sin(q(6));
            A62y = Lu*(cos(sigT56)-cos(sigT36))+Ll*cos(q(6));
        HTs.A02  = A0A*[cos(sigT36), sin(sigT36), 0, A62x;
                       -sin(sigT36), cos(sigT36), 0, A62y;
                                  0,           0, 1,    H;
                                  0,           0, 0,    1];
        % A06 * A63 = A03
            A63x = Lu*sin(sigT56)+Ll*sin(q(6));
            A63y = Lu*cos(sigT56)+Ll*cos(q(6));
        HTs.A03  = A0A*[cos(sigT46), sin(sigT46), 0, A63x;
                       -sin(sigT46), cos(sigT46), 0, A63y;
                                  0,           0, 1,    H;
                                  0,           0, 0,    1];
        % A06 * A6H = A0H
            A63x = Lu*sin(sigT56)+Ll*sin(q(6));
            A63y = Lu*cos(sigT56)+Ll*cos(q(6));
        HTs.A0H  = A0A*[cos(sigT46), sin(sigT46), 0, A63x;
                       -sin(sigT46), cos(sigT46), 0, A63y;
                                  0,           0, 1,   H/2;
                                  0,           0, 0,    1];
        % A06 * A64 = A04
            A64x = Lu*sin(sigT56)+Ll*sin(q(6));
            A64y = Lu*cos(sigT56)+Ll*cos(q(6));
        HTs.A04  = A0A*[cos(sigT56), sin(sigT56), 0, A64x;
                       -sin(sigT56), cos(sigT56), 0, A64y;
                                  0,           0, 1,    0;
                                  0,           0, 0,    1];
            A65x = Ll*sin(q(6));
            A65y = Ll*cos(q(6));
        HTs.A05  = A0A*[cos( q(6) ), sin( q(6) ), 0, A65x;
                       -sin( q(6) ), cos( q(6) ), 0, A65y;
                                  0,           0, 1,    0;
                                  0,           0, 0,    1];
        HTs.A06 = A0A;
    end
end