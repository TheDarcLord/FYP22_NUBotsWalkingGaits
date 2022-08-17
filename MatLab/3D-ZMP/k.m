function [xe, r0EL, r0ER, r0H] = k(q, index, model, params)
% k(q)  [2D Model] Forward Kinematic Model - FKM
%       
%       Returns:    [xe, TAA, Transforms] for an array of 'q'
%       xe:         End Effector Pose [X Y Z ϕ θ Ψ]'
%       TAA:        Transform from ANKLE to END EFFECTOR
%       Transforms: All other Homogenous Transforms
    
    %% HELPER VARS
    Rzyx   = @(Rz,Ry,Rx) ...
        [ cos(Rz)*cos(Ry), -sin(Rz)*cos(Rx)+cos(Rz)*sin(Ry)*sin(Rx),...
                    sin(Rz)*sin(Rx)+cos(Rz)*sin(Ry)*cos(Rx);
          sin(Rz)*cos(Ry),  cos(Rz)*cos(Rx)+sin(Rz)*sin(Ry)*sin(Rx),...
                   -cos(Rz)*sin(Rx)+sin(Rz)*sin(Ry)*cos(Rx);
                 -sin(Ry),  cos(Ry)*sin(Rx)                        ,...
                    cos(Ry)*cos(Rx)];

    %% LINK VARIABLES
    Ll     = params.fibula;     % Lower Leg
    Lu     = params.femur;      % Upper Leg
    H      = params.HipWidth;
    S      = params.ServoSize;  % SERVO DIST
    
    ABEL    = [Rzyx(model.r.r0Lg(6,index), ...
                    model.r.r0Lg(5,index), ...
                    model.r.r0Lg(4,index)),... 
                    model.r.r0Lg(1:3,index);  % LEFT Ankle Position from 
              zeros(1,3),           1]; %           0rigin in Global
    ABER    = [Rzyx(model.r.r0Rg(6,index), ...
                    model.r.r0Rg(5,index), ...
                    model.r.r0Rg(4,index)),... 
                    model.r.r0Rg(1:3,index);  % RIGHT Ankle Position from 
              zeros(1,3),           1]; %           0rigin in Global
    
    %% JOINT VARIABLES
    c1   = cos(q(1));
    s1   = sin(q(1));
    c2   = cos(q(2));
    s2   = sin(q(2));
    c23  = cos(q(2)+q(3));
    s23  = sin(q(2)+q(3));
    c234 = cos(q(2)+q(3)+q(4));
    s234 = sin(q(2)+q(3)+q(4));
    c34 = cos(q(3)+q(4));
    s34 = sin(q(3)+q(4));
    c4   = cos(q(4));
    s4   = sin(q(4));

    s5   = sin(q(5));
    c5   = cos(q(5));
    s6   = sin(q(6));
    c6   = cos(q(6));

    s7   = sin(q(7));
    c7   = cos(q(7));
    s8   = sin(q(8));
    c8   = cos(q(8));

    c9   = cos(q(9));
    s9   = sin(q(9));
    s90  = sin(q(9)  +q(10));
    c90  = cos(q(9)  +q(10));
    s901 = sin(q(9)  +q(10)  +q(11));
    c901 = cos(q(9)  +q(10)  +q(11));
    s101 = sin(q(10) +q(11));
    c101 = cos(q(10) +q(11));
    s11  = sin(q(11));
    c11  = cos(q(11));
    c12  = cos(q(12));
    s12  = sin(q(12));
    %% HOMOGENOUS TRANSFORM
    if params.mode ==  1        % LEFT FIXED
        TB0   = [0,0,1,0; 0,1,0,0;
                -1,0,0,0; 0,0,0,1];
        % INVERTIBLE !!!
        A04 = [c1, -s1*c234, -s1*s234, -s1*(Lu*c23 +Ll*c2 +S*(c234+1));
               s1,  c1*c234,  c1*s234,  c1*(Lu*c23 +Ll*c2 +S*(c234+1));
                0,    -s234,     c234,  -1*(Lu*s23 +Ll*s2 +S*s234);
                0,        0,        0,   1];
        A46 = [c5*c6, -c5*s6,  s5, (H*c5*c6 -S*s5);
               s5*c6, -s5*s6, -c5, (H*s5*c6 +S*c5);
                  s6,     c6,   0, (H*s6);
                   0,      0,   0, 1];
        A68 = [-s7, -c7*s8, -c7*c8, S*c7*s8;
                c7, -s7*s8, -c8*s7, S*s7*s8;
                 0,    -c8,     s8, S*(c8+1);
                 0,      0,      0, 1];
        A812 = [-s901*s12, -s901*c12, c901,  Ll*s90+Lu*s9+S*s901;
                 c901*s12,  c901*c12, s901, -Ll*c90-Lu*c9-S*c901;
                     -c12,       s12,    0,  0;
                        0,         0,    0,  1];
        % INVERTIBLE !!!
        T12B  = [0,0,-1,0; 0,1,0,0;
                 1,0, 0,0; 0,0,0,1];
        
        AB6  = ABEL*TB0*A04*A46;

        ABH  = AB6*[eye(3),[-H/2;0;0];
                     0,0,0,        1];

        ABER = AB6*A68*A812*T12B;
        TAE  = ABER;
    
    elseif params.mode == -1     % RIGHT FIXED
        T12B  = [0,0,-1,0; 0,1,0,0;
                 1,0, 0,0; 0,0,0,1];
        % INVERTIBLE !!!
        A40 = [     c1,      s1,     0,  0;
              -c234*s1, c234*c1, -s234, -1*(Ll*c34 +Lu*c4 +S*(c234+1));
              -s234*s1, s234*c1,  c234, -1*(Ll*s34 +Lu*s4 +S*s234);
                     0,       0,     0,  1];
        A64 = [c5*c6,  c6*s5, s6, -H;
              -c5*s6, -s5*s6, c6,  0;
                  s5,    -c5,  0,  S;
                   0,      0,  0,  1];
        A86 = [  -s7,     c7,   0,  0;
              -c7*s8, -s7*s8, -c8,  S*(c8 + 1);
              -c7*c8, -c8*s7,  s8, -S*s8;
                   0,      0,   0,  1];
        A128 = [-s12*s901, s12*c901, -c12, (Lu*c101 +Ll*c11 +S)*s12;
                -c12*s901, c12*c901,  s12, (Lu*c101 +Ll*c11 +S)*c12;
                     c901,     s901,    0, (Lu*s101 +Ll*s11);
                        0,        0,    0, 1];
        % INVERTIBLE !!!
        TB0   = [0,0,1,0; 0,1,0,0;
                -1,0,0,0; 0,0,0,1];
        
        AB6  = ABER*TB0*A128*A86;
        ABH  = AB6*[eye(3),[-H/2;0;0];
                     0,0,0,        1];
        ABEL = AB6*A64*A40*T12B;
        TAE  = ABEL;
    end
    
    %% End Effector Parameterisation X Y Z R₂Ψ Rᵧθ Rₓϕ
    % R₁₁:  cθ₁cθ₂  Ψ = atan2( R₂₁, R₁₁)               Z
    % R₂₁:  sθ₁cθ₂  θ = atan2(-R₃₁, SQRT(R₃₂² + R₃₃²)) Y
    % R₃₁: -sθ₂     ϕ = atan2( R₃₂, R₃₃)               X   
    % R₃₂:  0
    % R₃₃:  cθ₂
    
    parameteriseXYZ = @(A) ...
        [A(1:3,4);                                  % X Y Z
         atan2( A(3,2), A(3,3));                    % ϕ R(x)
         atan2(-A(3,1), sqrt(A(3,2)^2 + A(3,3)^2)); % θ R(y) 
         atan2( A(2,1), A(1,1))];                   % Ψ R(z)
    xe   = parameteriseXYZ(TAE);
    r0EL = parameteriseXYZ(ABEL);
    r0ER = parameteriseXYZ(ABER);
    r0H  = parameteriseXYZ(ABH);
end