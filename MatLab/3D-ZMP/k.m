function [xe, r0EL, r0ER, r0H] = k_1(q, index, model, params)
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

    %% HOMOGENOUS TRANSFORM
    if params.mode ==  1        % LEFT FIXED
        TB0   = [0,0,1,0; 0,1,0,0;
                -1,0,0,0; 0,0,0,1];
        % INVERTIBLE !!!
        A01   = [0, -sin(q1), -cos(q1), -S*sin(q1);
                 0,  cos(q1), -sin(q1),  S*cos(q1);
                 1,        0,        0,          0;
                 0,        0,        0,          1];
        A12   = [cos(q2), -sin(q2), 0, -Ll*sin(q2);
                 sin(q2),  cos(q2), 0,  Ll*cos(q2);
                       0,        0, 1,           0;
                       0,        0, 0,           1];
        A23   = [cos(q3), -sin(q3), 0, -Lu*sin(q3);
                 sin(q3),  cos(q3), 0,  Lu*cos(q3);
                       0,        0, 1,           0;
                       0,        0, 0,           1];
        A34   = [0, -sin(q4), cos(q4), -S*sin(q4);
                 0,  cos(q4), sin(q4),  S*cos(q4);
                -1,        0,       0,          0;
                 0,        0,       0,          1];
        A45   = [cos(q5), 0,  sin(q5), -S*sin(q5);
                 sin(q5), 0, -cos(q5),  S*cos(q5);
                       0, 1,        0,          0;
                       0, 0,        0,          1];
        A56   = [cos(q6), -sin(q6), 0, H*cos(q6);
                 sin(q6),  cos(q6), 0, H*sin(q6);
                       0,        0, 1,         0;
                       0,        0, 0,         1];
        A67   = [cos(q7),  0, -sin(q7), 0;
                 sin(q7),  0,  cos(q7), 0;
                       0, -1,        0, S;
                       0,  0,        0, 1];
        A78   = [0, -sin(q8), -cos(q8),  S*sin(q8);
                 0,  cos(q8), -sin(q8), -S*cos(q8);
                 1,        0,        0,          0;
                 0,        0,        0,          1];
        A89   = [cos(q9), -sin(q9), 0,  Lu*sin(q9);
                 sin(q9),  cos(q9), 0, -Lu*cos(q9);
                       0,        0, 1,           0;
                       0,        0, 0,           1];
        A910  = [cos(q10), -sin(q10), 0,  Ll*sin(q10);
                 sin(q10),  cos(q10), 0, -Ll*cos(q10);
                        0,         0, 1,            0;
                        0,         0, 0,            1];
        A1011 = [ 0, -sin(q11), cos(q11),  S*sin(q11);
                  0,  cos(q11), sin(q11), -S*cos(q11);
                 -1,         0,        0,           0;
                  0,         0,        0,           1];
        A1112 = [cos(q12), -sin(q12), 0, 0;
                 sin(q12),  cos(q12), 0, 0;
                        0,         0, 1, 0;
                        0,         0, 0, 1];
        % INVERTIBLE !!!
        T12B  = [0,0,-1,0; 0,1,0,0;
                 1,0, 0,0; 0,0,0,1];
        
        AB6  = ABEL*TB0*A01*A12*A23*A34*A45*A56;
        ABH  = AB6*[eye(3),[-H/2;0;0];
                     0,0,0,        1];
        ABER = AB6*A67*A78*A89*A910*A1011*A1112*T12B;
        TAE  = ABER;
    
    elseif params.mode == -1     % RIGHT FIXED
        TB0   = [0,0,1,0; 0,1,0,0;
                -1,0,0,0; 0,0,0,1];
        % INVERTIBLE !!!
        A01   = [0, -sin(q1), -cos(q1), -S*sin(q1);
                 0,  cos(q1), -sin(q1),  S*cos(q1);
                 1,        0,        0,          0;
                 0,        0,        0,          1];
        A12   = [cos(q2), -sin(q2), 0, -Ll*sin(q2);
                 sin(q2),  cos(q2), 0,  Ll*cos(q2);
                       0,        0, 1,           0;
                       0,        0, 0,           1];
        A23   = [cos(q3), -sin(q3), 0, -Lu*sin(q3);
                 sin(q3),  cos(q3), 0,  Lu*cos(q3);
                       0,        0, 1,           0;
                       0,        0, 0,           1];
        A34   = [0, -sin(q4), cos(q4), -S*sin(q4);
                 0,  cos(q4), sin(q4),  S*cos(q4);
                -1,        0,       0,          0;
                 0,        0,       0,          1];
        A45   = [cos(q5), 0,  sin(q5), -S*sin(q5);
                 sin(q5), 0, -cos(q5),  S*cos(q5);
                       0, 1,        0,          0;
                       0, 0,        0,          1];
        A56   = [cos(q6), -sin(q6), 0, H*cos(q6);
                 sin(q6),  cos(q6), 0, H*sin(q6);
                       0,        0, 1,         0;
                       0,        0, 0,         1];
        A67   = [cos(q7),  0, -sin(q7), 0;
                 sin(q7),  0,  cos(q7), 0;
                       0, -1,        0, S;
                       0,  0,        0, 1];
        A78   = [0, -sin(q8), -cos(q8),  S*sin(q8);
                 0,  cos(q8), -sin(q8), -S*cos(q8);
                 1,        0,        0,          0;
                 0,        0,        0,          1];
        A89   = [cos(q9), -sin(q9), 0,  Lu*sin(q9);
                 sin(q9),  cos(q9), 0, -Lu*cos(q9);
                       0,        0, 1,           0;
                       0,        0, 0,           1];
        A910  = [cos(q10), -sin(q10), 0,  Ll*sin(q10);
                 sin(q10),  cos(q10), 0, -Ll*cos(q10);
                        0,         0, 1,            0;
                        0,         0, 0,            1];
        A1011 = [ 0, -sin(q11), cos(q11),  S*sin(q11);
                  0,  cos(q11), sin(q11), -S*cos(q11);
                 -1,         0,        0,           0;
                  0,         0,        0,           1];
        A1112 = [cos(q12), -sin(q12), 0, 0;
                 sin(q12),  cos(q12), 0, 0;
                        0,         0, 1, 0;
                        0,         0, 0, 1];
        % INVERTIBLE !!!
        T12B  = [0,0,-1,0; 0,1,0,0;
                 1,0, 0,0; 0,0,0,1];
        
        AB6  = ABER*inv(T12B)*inv(A1112)*inv(A1011)*inv(A910)* ...
                inv(A89)*inv(A78)*inv(A67);
        ABH  = AB6*[eye(3),[-H/2;0;0];
                     0,0,0,        1];
        ABEL = AB6*inv(A56)*inv(A45)*inv(A34)*inv(A23)*inv(A12)*inv(A01)*inv(TB0);
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