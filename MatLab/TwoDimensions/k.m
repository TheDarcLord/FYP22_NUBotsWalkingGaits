function [xe, T16, Transforms] = k(q, params)
% k(q)  [2D Model] Forward Kinematic Model - FKM
%       
%       Returns: [T16 Transforms] for an array of 'q'
%       T16:        Transform from ONE to END EFFECTOR
%       Transforms: All other Homogenous Transforms
    t1     = q(1); % θ₁
    t2     = q(2); % θ₂
    t3     = q(3); % θ₂
    t4     = q(4); % θ₃
    t5     = q(5); % θ₃
    t6     = q(6); % θ₃

    L1     = params.L1;
    L2     = params.L2;
    H      = params.HipWidth;

    RZ    = @(psi) ...
            [cos(psi) -sin(psi)  0  0;
             sin(psi)  cos(psi)  0  0;
              0        0         1  0;
              0        0         0  1];
    T     = @(x, y, z) ...
            [eye(3)     [x;y;z];
             zeros(1,3)      1];

    T16 = RZ(t1)*T(0,L1,0)*RZ(t2)*T(0,L2,0)*RZ(t3)*T(0,0,-H)* ... TO HIP
          RZ(t4)*T(0,-L2,0)*RZ(t5)*T(0,-L1,0)*RZ(t6);
    
    % Ψ = atan2( R₃₂, R₃₃)
    % θ = atan2( R₂₁, R₁₁)
    % ϕ = atan2(-R₃₁, SQRT(R₃₂² + R₃₃²))
    % 
    % R₁₁:  cθ₁cθ₂
    % R₂₁:  sθ₁cθ₂
    % R₃₁: -sθ₂
    % R₃₂:  0
    % R₃₃:  cθ₂
    phi     = 0;
    theta   = 0;
    psi     = 0;
    if params.orientation == 1
        phi =   atan2( T16(3,2), T16(3,3) );
        theta = atan2(-T16(3,1), sqrt( T16(3,2)^2 + T16(3,3)^2 ) );
        psi =   atan2( T16(2,1), T16(1,1) );
    end
    
    xe = [T16(1:3,4); % X Y Z
          phi;        % ϕ
          theta;      % θ
          psi];       % Ψ
    
    
    Transforms.A12 = RZ(t1)*T(0,L1,0);
    Transforms.A23 = RZ(t2)*T(0,L2,0);
    Transforms.A34 = RZ(t3)*T(0,0,-H);
    Transforms.A45 = RZ(t4)*T(0,-L2,0);
    Transforms.A56 = RZ(t5)*T(0,-L1,0);

    Transforms.A13 = Transforms.A12*Transforms.A23;
    Transforms.A14 = Transforms.A13*Transforms.A34;
    Transforms.A15 = Transforms.A14*Transforms.A45;
    Transforms.A16 = Transforms.A15*Transforms.A56;

end