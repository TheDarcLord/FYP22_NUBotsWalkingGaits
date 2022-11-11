function [xe, T03, Transforms] = k(q, params)
% k(q)  Forward Kinematic Model.
%       Returns: [T03 Transforms] for an array of 'q'
%       T03:        Transform from ZERO to END EFFECTOR
%       Transforms: All other Homogenous Transforms
    t1     = q(1); % θ₁
    t2     = q(2); % θ₂
    d2     = q(3); % d₂
    d3     = q(4); % d₃

    T03 = [cos(t1)*cos(t2), -sin(t1),  cos(t1)*sin(t2),  cos(t1)*sin(t2)*d3 - sin(t1)*d2;
           sin(t1)*cos(t2),  cos(t1),  sin(t1)*sin(t2),  sin(t1)*sin(t2)*d3 + cos(t1)*d2;
                  -sin(t2),        0,          cos(t2),                       cos(t2)*d3;
                         0,        0,                0,                                1];
    
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
        phi =   atan2( T03(3,2), T03(3,3) );
        theta = atan2(-T03(3,1), sqrt( T03(3,2)^2 + T03(3,3)^2 ) );
        psi =   atan2( T03(2,1), T03(1,1) );
    end
    
    xe = [T03(1:3,4); % X Y Z
          phi;        % ϕ
          theta;      % θ
          psi];       % Ψ

    Transforms.A01 = ...
    [cos(t1),  0, -sin(t1), 0;
     sin(t1),  0,  cos(t1), 0;
           0, -1,        0, 0;
           0   0,        0, 1];
    Transforms.A12 = ...
    [cos(t2),  0,  sin(t2), 0;
     sin(t2),  0, -cos(t2), 0;
           0,  1,        0, d2;
           0   0,        0, 1];
    Transforms.A23 = ...
    [1,  0, 0, 0;
     0,  1, 0, 0;
     0,  0, 1, d3;
     0   0, 0, 1];
    Transforms.A02 = Transforms.A01*Transforms.A12;

end