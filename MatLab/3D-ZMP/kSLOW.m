function [HTs] = kSLOW(q, index, model, params)
% k(q)  [2D Model] Forward Kinematic Model - FKM
%       
%       Returns:    [xe, TAA, Transforms] for an array of 'q'
%       xe:         End Effector Pose [X Y Z ϕ θ Ψ]'
%       TAA:        Transform from ANKLE to END EFFECTOR
%       Transforms: All other Homogenous Transforms
    
    %% HELPER VARS
    AE1  = [0  0  1  0;   %
            0  1  0  0;   %
           -1  0  0  0;   %
            0  0  0  1];  %
    AE12 = [0  0  1  0;   %
            0  1  0  0;   %
           -1  0  0  0;   %
            0  0  0  1];  %
    ROW4 = [0  0  0  1];

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
    
    A0EL    = [Rzyx(model.r.r0Lg(6,index), ...
                    model.r.r0Lg(5,index), ...
                    model.r.r0Lg(4,index)),... 
                    model.r.r0Lg(1:3,index);  % LEFT Ankle Position from 
              zeros(1,3),           1]; %           0rigin in Global
    A0ER    = [Rzyx(model.r.r0Rg(6,index), ...
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
    HTs.A0EL = A0EL;
    HTs.A01 = HTs.A0EL * AE1;

    A12x = -S*sin(q1);
    A12y =  S*cos(q1);
    A12z =  0;
    A12R = [0, -sin(q1), -cos(q1);
            0,  cos(q1), -sin(q1);
            1,        0,        0];
    A12  = [A12R, [A12x A12y A12z]'; ROW4];
    HTs.A02 = HTs.A01 * A12;

    A23x = -Ll*sin(q2);
    A23y =  Ll*cos(q2);
    A23z =  0;
    A23R = [cos(q2), -sin(q2), 0;
            sin(q2),  cos(q2), 0;
                  0,        0, 1];
    A23  = [A23R, [A23x A23y A23z]'; ROW4];
    HTs.A03 = HTs.A02 * A23;

    A34x = -Lu*sin(q3);
    A34y =  Lu*cos(q3);
    A34z =  0;
    A34R = [cos(q3), -sin(q3), 0;
            sin(q3),  cos(q3), 0;
                  0,        0, 1];
    A34  = [A34R, [A34x A34y A34z]'; ROW4];
    HTs.A04 = HTs.A03 * A34;

    A45x = -S*sin(q4);
    A45y =  S*cos(q4);
    A45z =  0;
    A45R = [0, -sin(q4),  cos(q4);
            0,  cos(q4),  sin(q4);
           -1,        0,        0];
    A45  = [A45R, [A45x A45y A45z]'; ROW4];
    HTs.A05 = HTs.A04 * A45;

    A56x = -S*sin(q5);
    A56y =  S*cos(q5);
    A56z =  0;
    A56R = [cos(q5), 0,  sin(q5);
            sin(q5), 0, -cos(q5);
                  0, 1,        0];
    A56  = [A56R, [A56x A56y A56z]'; ROW4];
    HTs.A06 = HTs.A05 * A56;

    % ___ MID-BEGIN ___ %
    A6Hx = H/2;
    A6Hy =   0;
    A6Hz =   0;
    A6HR = eye(3);
    A6H  = [A6HR, [A6Hx A6Hy A6Hz]'; ROW4];
    HTs.A0H =  HTs.A06 * A6H;
    % ___  MID-END  ___ %

    HTs.A0ER = A0ER;
    HTs.A012 = HTs.A0ER * AE12;

    A1211x =  S*sin(q12);
    A1211y =  S*cos(q12);
    A1211z =  0;
    A1211R = [0, sin(q12), -cos(q12);
              0, cos(q12),  sin(q12);
              1,        0,         0];
    A1211  = [A1211R, [A1211x A1211y A1211z]'; ROW4];
    HTs.A011  = HTs.A012 * A1211;

    A1110x =  Ll*sin(q11);
    A1110y =  Ll*cos(q11);
    A1110z =  0;
    A1110R = [cos(q11), sin(q11), 0;
             -sin(q11), cos(q11), 0;
                     0,        0, 1];
    A1110  = [A1110R, [A1110x A1110y A1110z]'; ROW4];
    HTs.A010  = HTs.A011 * A1110;

    A109x =  Lu*sin(q10);
    A109y =  Lu*cos(q10);
    A109z =  0;
    A109R = [cos(q10), sin(q10), 0;
            -sin(q10), cos(q10), 0;
                   0,       0,   1];
    A109  = [A109R, [A109x A109y A109z]'; ROW4];
    HTs.A09  = HTs.A010 * A109;

    A98x = S*sin(q9);
    A98y = S*cos(q9);
    A98z = 0;
    A98R = [0, sin(q9),  cos(q9);
            0, cos(q9), -sin(q9);
           -1,       0,       0];
    A98  = [A98R, [A98x A98y A98z]'; ROW4];
    HTs.A08  = HTs.A09 * A98;

    A87x =  S*sin(q8);
    A87y =  S*cos(q8);
    A87z =  0;
    A87R = [cos(q8), 0, -sin(q8);
           -sin(q8), 0, -cos(q8);
                  0, 1,        0];
    A87  = [A87R, [A87x A87y A87z]'; ROW4];
    HTs.A07  = HTs.A08 * A87;
end