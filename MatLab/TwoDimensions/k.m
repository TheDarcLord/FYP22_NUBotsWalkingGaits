function [xe, T16, HTs] = k(q, params)
% k(q)  [2D Model] Forward Kinematic Model - FKM
%       
%       Returns: [T16 Transforms] for an array of 'q'
%       T16:        Transform from ONE to END EFFECTOR
%       Transforms: All other Homogenous Transforms

    t1          = q(1);          % θ₁     ankle
    t2          = q(2);          % θ₂     knee
    t3          = q(3);          % θ₃     hip
    t4          = q(4);          % θ₄     hip
    t5          = q(5);          % θ₅     knee
    t6          = q(6);          % θ₆     ankle
    sigT        = sum(q);        % Σθᵢ    i=1:6
    sigT15      = sum(q(1:5));   % Σθᵢ    i=1:5
    sigT14      = sum(q(1:4));   % Σθᵢ    i=1:4
    sigT12      = sum(q(1:2));   % Σθᵢ    i=1:2

    L_lower     = params.fibula;
    L_upper     = params.femur;
    H           = params.HipWidth;

    T16x = L_upper*sin(sigT14) - L_upper*sin(sigT12) - L_lower*sin(t1) + L_lower*sin(sigT15);
    T16y = L_upper*cos(sigT12) - L_upper*cos(sigT14) + L_lower*cos(t1) - L_lower*cos(sigT15);
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

    RZ    = @(psi) ...
            [cos(psi) -sin(psi)  0  0;
             sin(psi)  cos(psi)  0  0;
              0        0         1  0;
              0        0         0  1];
    T     = @(x, y, z) ...
            [eye(3)     [x;y;z];
             zeros(1,3)      1];

    HTs.A01 = T(0,0,0);
        HTs.A12 = RZ(t1)*T(0,L_lower,0);
        HTs.A23 = RZ(t2)*T(0,L_upper,0);
        HTs.A34 = RZ(t3)*T(0,0,-H);
        HTs.A45 = RZ(t4)*T(0,-L_upper,0);
        HTs.A56 = RZ(t5)*T(0,-L_lower,0);
    HTs.A02 = HTs.A01*HTs.A12;
    HTs.A03 = HTs.A02*HTs.A23;
    HTs.A04 = HTs.A03*HTs.A34;
    HTs.A05 = HTs.A04*HTs.A45;
    HTs.A06 = HTs.A05*HTs.A56;

end