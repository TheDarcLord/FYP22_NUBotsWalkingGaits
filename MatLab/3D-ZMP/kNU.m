function [xe, r0EL, r0ER, r0H] = kNU(q, index, model, params)
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
    H    = params.HipWidth;
    h2a  = params.heel2ankle;
    a2k  = params.ankle2knee;
    k2h  = params.knee2hip;
    h2w  = params.hip2waist;
    Ll   = params.fibula;     % Lower Leg
    Lu   = params.femur;      % Upper Leg
    S    = params.ServoSize;  % SERVO DIST

    T12B  = [0,0,-1,0; 0,1,0,-h2a;
             1,0, 0,0; 0,0,0,  1];
    TB0   = [0,0, 1,0; 0,1,0, h2a;
            -1,0, 0,0; 0,0,0,  1];
    
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
    c3   = cos(q(3));
    s3   = sin(q(3));
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
    c10  = cos(q(10));
    s10  = sin(q(10));
    s901 = sin(q(9)  +q(10)  +q(11));
    c901 = cos(q(9)  +q(10)  +q(11));
    s101 = sin(q(10) +q(11));
    c101 = cos(q(10) +q(11));
    s11  = sin(q(11));
    c11  = cos(q(11));
    c12  = cos(q(12));
    s12  = sin(q(12));
    %% HOMOGENOUS TRANSFORM
    if params.mode ==  1        % RIGHT FIXED
        % INVERTIBLE !!!
        x04 = -s1*(a2k(2)*c2 +a2k(1)*s2)...
              -c1*(a2k(3) +k2h(3))...
              -c2*s1*(k2h(2)*c3 +k2h(1)*s3)...
              -s1*s2*(k2h(1)*c3 -k2h(2)*s3)...
              -h2w(1)*s1*(c23*s4 +s23*c4);
        y04 =  c1*(a2k(2)*c2 +a2k(1)*s2)...
              -s1*(a2k(3) +k2h(3)) ...
              +c1*c2*(k2h(2)*c3 +k2h(1)*s3)...
              +c1*s2*(k2h(1)*c3 -k2h(2)*s3)...
              +h2w(1)*c1*(c23*s4 +s23*c4);
        z04 = c234*h2w(1) -a2k(2)*s2 +c23*k2h(1) -s23*k2h(2) + a2k(1)*c2;
        A04 = [c1, -s1*c234, -s1*s234, x04;
               s1,  c1*c234,  c1*s234, y04;
                0,    -s234,     c234, z04;
                0,        0,        0,   1];

        A46 = [c5*c6, -c5*s6,  s5, ( h2w(3)*s5 +H*c5*c6);
               c6*s5, -s5*s6, -c5, (-h2w(3)*c5 +H*s5*c6);
                  s6,     c6,   0, ( h2w(2)    +H*s6);
                   0,      0,   0, 1];
        
        A68 = [  -s7, -c7*s8, -c7*c8,  s7*(h2w(1) + h2w(2));
                  c7, -s7*s8, -c8*s7, -c7*(h2w(1) + h2w(2));
                   0,    -c8,     s8, -h2w(3);
                   0,      0,      0, 1];

        x812 =  k2h(2)*s9 -k2h(1)*c9 -c90*a2k(1) +s90*a2k(2);
        y812 = -k2h(2)*c9 -k2h(1)*s9 -c90*a2k(2) -s90*a2k(1);
        z812 = -a2k(3) -k2h(3);
        A812 = [-s901*s12, -s901*c12, c901, x812;
                 c901*s12,  c901*c12, s901, y812;
                     -c12,       s12,    0, z812;
                        0,         0,    0,   1];
        % INVERTIBLE !!!

        AB6  = ABEL*TB0*A04*A46;

        ABH  = AB6*[eye(3),[-H/2;0;0];
                     0,0,0,        1];

        ABER = AB6*A68*A812*T12B;
        TAE  = ABER;
    
    elseif params.mode == -1     % LEFT FIXED
        % INVERTIBLE !!!
        x40 =  a2k(3) +k2h(3);
        y40 =  a2k(1)*s34 -a2k(2)*c34 -k2h(2)*c4 +k2h(1)*s4;
        z40 = -h2w(1) -a2k(1)*c34 -a2k(2)*s34 -k2h(1)*c4 -k2h(2)*s4;
        A40 = [     c1,      s1,     0, x40;
              -c234*s1, c234*c1, -s234, y40;
              -s234*s1, s234*c1,  c234, z40;
                     0,       0,     0,   1];

        A64 = [c5*c6,  c6*s5, s6, -(H+h2w(2)*s6);
              -c5*s6, -s5*s6, c6, -h2w(2)*c6;
                  s5,    -c5,  0, -h2w(3);
                   0,      0,  0,  1];

        A86 = [  -s7,     c7,   0,  h2w(1) + h2w(2);
              -c7*s8, -s7*s8, -c8, -h2w(3)*c8;
              -c7*c8, -c8*s7,  s8,  h2w(3)*s8;
                   0,      0,   0,  1];
        
        x128 = a2k(2)*c11*s12 -a2k(3)*c12 -k2h(3)*c12 ...
              -a2k(1)*s11*s12 +k2h(2)*c10*c11*s12 ...
              -k2h(1)*c10*s11*s12 -k2h(1)*c11*s10*s12 -k2h(2)*s10*s11*s12;
        y128 = a2k(2)*c11*c12 +a2k(3)*s12 +k2h(3)*s12 ...
              -a2k(1)*c12*s11 +k2h(2)*c10*c11*c12 ...
              -k2h(1)*c10*c12*s11 -k2h(1)*c11*c12*s10 ...
              -k2h(2)*c12*s10*s11;
        z128 = k2h(1)*c101 +k2h(2)*s101 +a2k(1)*c11 +a2k(2)*s11;
        A128 = [-s12*s901, s12*c901, -c12, x128;
                -c12*s901, c12*c901,  s12, y128;
                     c901,     s901,    0, z128;
                        0,        0,    0, 1];
        % INVERTIBLE !!!
        
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