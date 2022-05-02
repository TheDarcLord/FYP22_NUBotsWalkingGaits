function [xe, TAE, HTs] = k(q, params)
% k(q)  [2D Model] Forward Kinematic Model - FKM
%       
%       Returns:    [xe, TAA, Transforms] for an array of 'q'
%       xe:         End Effector Pose [X Y Z ϕ θ Ψ]'
%       TAA:        Transform from ANKLE to END EFFECTOR
%       Transforms: All other Homogenous Transforms
    
    %% HELPER FUNCTIONS
    T  = @(x,y,z)   [eye(3), [x;y;z];
                      0,0,0,      1];
    Rz = @(psi)     [cos(psi), -sin(psi), 0, 0;
                     sin(psi),  cos(psi), 0, 0;
                            0,         0, 1, 0;
                            0,         0, 0, 1];
    Ry = @(theta)   [cos(theta), 0, sin(theta), 0;
                              0, 1,          0, 0;
                    -sin(theta), 0, cos(theta), 0;
                              0, 0,          0, 1];
    Rx = @(phi)     [1,        0,         0, 0;
                     0, cos(phi), -sin(phi), 0;
                     0, sin(phi),  cos(phi), 0;
                     0,        0,         0, 1];

    %% LINK VARIABLES
    Ll     = params.fibula;     % Lower Leg
    Lu     = params.femur;      % Upper Leg
    H      = params.HipWidth;
    S      = 0.05;              % SERVO DIST
    
    A0L    = [eye(3), params.r0Lg;  % LEFT Ankle Position from 
              zeros(1,3),       1]; %      0rigin in Global
    A0R    = [eye(3), params.r0Rg;  % RIGHT Ankle Position from 
              zeros(1,3),       1]; %       0rigin in Global
    
    %% JOINT VARIABLES
    q1  = q(1);     % θ₁
    q2  = q(2);     % θ₂
    q3  = q(3);     % θ₃
    q4  = q(4);     % θ₄
    q5  = q(5);     % θ₅
    q6  = q(6);     % θ₆
    q7  = q(7);     % θ₇
    q8  = q(8);     % θ₈
    q9  = q(9);     % θ₉
    q10 = q(10);    % θ₁₀
    q11 = q(11);    % θ₁₁
    q12 = q(12);    % θ₁₂

%     sigT        = sum(q);        % Σθᵢ    i=1:6
%     sigT15      = sum(q(1:5));   % Σθᵢ    i=1:5
%     sigT14      = sum(q(1:4));   % Σθᵢ    i=1:4
%     sigT13      = sum(q(1:3));   % Σθᵢ    i=1:3
%     sigT12      = sum(q(1:2));   % Σθᵢ    i=1:2
% 
%     sigT26      = sum(q(2:6));   % Σθᵢ    i=2:6
%     sigT36      = sum(q(3:6));   % Σθᵢ    i=3:6
%     sigT46      = sum(q(4:6));   % Σθᵢ    i=4:6
%     sigT56      = sum(q(5:6));   % Σθᵢ    i=5:6

    %% HOMOGENOUS TRANSFORM
    if params.mode == -1        % LEFT FIXED
        T1_12 = Rz( q1)*Ry( pi/2)*T(0,  S,  0)  ... A12
               *Rz( q2)*Ry(-pi/2)*T(0, Ll,  0)  ... A23
               *Rz( q3)          *T(0, Lu,  0)  ... A34
               *Rz( q4)*Ry( pi/2)*T(0,  S,  0)  ... A45
               *Rz( q5)*Rx( pi/2)*T(0,  0, -S)  ... A56
               *Rz( q6)          *T(H,  0,  0)  ... A67
               *Rz( q7)*Rx(-pi/2)*T(0, -S,  0)  ... A78
               *Rz( q8)*Ry(-pi/2)*T(0, -S,  0)  ... A89
               *Rz( q9)          *T(0,-Lu,  0)  ... A9_10
               *Rz(q10)*Ry( pi/2)*T(0,-Ll,  0)  ... A10_11
               *Rz(q11)*Ry(-pi/2)*T(0, -S,  0)  ... A11_12
               *Rz(q12);
        TAE = A0L*T1_12;
    elseif params.mode == 0     % BOTH FIXED
        % Need to locate the CoM !
        % Shift the waist instead...?
        
    elseif params.mode == 1     % RIGHT FIXED
       T12_1 = Rz(-q12)*Ry( pi/2)*T(0,  S,  0)  ... A12_11
              *Rz(-q11)*Ry(-pi/2)*T(0, Ll,  0)  ... A11_10
              *Rz(-q10)          *T(0, Lu,  0)  ... A10_9
              *Rz(-q9) *Ry( pi/2)*T(0,  S,  0)  ... A98
              *Rz(-q8) *Rx( pi/2)*T(0,  0, -S)  ... A87
              *Rz(-q7)           *T(-H,  0,  0)  ... A76
              *Rz(-q6) *Rx(-pi/2)*T(0, -S,  0)  ... A65
              *Rz(-q5) *Ry(-pi/2)*T(0, -S,  0)  ... A54
              *Rz(-q4)           *T(0,-Lu,  0)  ... A43
              *Rz(-q3) *Ry( pi/2)*T(0,-Ll,  0)  ... A32
              *Rz(-q2) *Ry(-pi/2)*T(0, -S,  0)  ... A21
              *Rz(-q1);
       TAE = A0R*T12_1;
    end

    %% Homogeneous Transforms
    if params.mode == -1
        HTs.A01  = A0L;
        % A0A * A12 = A02
        HTs.A02  = HTs.A01 *Rz( q1)*Ry( pi/2)*T(0,  S,  0);
        % A0A * A13 = A03
        HTs.A03  = HTs.A02 *Rz( q2)*Ry(-pi/2)*T(0, Ll,  0);
        % A0A * A14 = A04
        HTs.A04  = HTs.A03 *Rz( q3)          *T(0, Lu,  0);
        % A0A * A15 = A05
        HTs.A05  = HTs.A04 *Rz( q4)*Ry( pi/2)*T(0,  S,  0);
        % A0A * T16 = A06
        HTs.A06 =  HTs.A05 *Rz( q5)*Rx( pi/2)*T(0,  0, -S);

        % A0A * T1H = A0H
        HTs.A0H =  HTs.A06                   *T(H/2, 0,  0);
        
        % A0A * T17 = A07
        HTs.A07 =  HTs.A06 *Rz( q6)          *T(H,  0,  0);
        % A0A * T18 = A08
        HTs.A08 =  HTs.A07 *Rz( q7)*Rx(-pi/2)*T(0, -S,  0);
        % A0A * T19 = A09
        HTs.A09 =  HTs.A08 *Rz( q8)*Ry(-pi/2)*T(0, -S,  0);
        % A0A * T1_10 = A010
        HTs.A010 = HTs.A09 *Rz( q9)          *T(0,-Lu,  0);
        % A0A * T1_11 = A011
        HTs.A011 = HTs.A010*Rz(q10)*Ry( pi/2)*T(0,-Ll,  0);
        % A0A * T1_12 = A012
        HTs.A012 = HTs.A011*Rz(q11)*Ry(-pi/2)*T(0, -S,  0);
    elseif params.mode == 0
        % Adjust TAE
        rCM = rCoM(HTs,params);
        TAE = [eye(3),    [rCM(1);0;rCM(3)];  % rCoM X & Z only
               zeros(1,3),               1];  % Y vertical = 0
    elseif params.mode == 1
        HTs.A012  = A0R;
        % A0A * A12 = A02
        HTs.A011  = HTs.A012 *Rz(-q12)*Ry( pi/2)*T(0,  S,  0);
        % A0A * A13 = A03
        HTs.A010  = HTs.A011 *Rz(-q11)*Ry(-pi/2)*T(0, Ll,  0);
        % A0A * A14 = A04
        HTs.A09  = HTs.A010 *Rz(-q10)          *T(0, Lu,  0);
        % A0A * A15 = A05
        HTs.A08  = HTs.A09 *Rz(-q9)*Ry( pi/2)*T(0,  S,  0);
        % A0A * T16 = A06
        HTs.A07 =  HTs.A08 *Rz(-q8)*Rx( pi/2)*T(0,  0, -S);

        % A0A * T1H = A0H
        HTs.A0H =  HTs.A07                   *T(-H/2, 0,  0);
        
        % A0A * T17 = A07
        HTs.A06 =  HTs.A07 *Rz(-q7)          *T(-H,  0,  0);
        % A0A * T18 = A08
        HTs.A05 =  HTs.A06 *Rz(-q6)*Rx(-pi/2)*T(0, -S,  0);
        % A0A * T19 = A09
        HTs.A04 =  HTs.A05 *Rz(-q5)*Ry(-pi/2)*T(0, -S,  0);
        % A0A * T1_10 = A010
        HTs.A03 = HTs.A04 *Rz(-q4)          *T(0,-Lu,  0);
        % A0A * T1_11 = A011
        HTs.A02 = HTs.A03*Rz(-q3)*Ry( pi/2)*T(0,-Ll,  0);
        % A0A * T1_12 = A012
        HTs.A01 = HTs.A02*Rz(-q2)*Ry(-pi/2)*T(0, -S,  0);
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
    
    xe = [xe; HTs.A0H(2,4)];
end