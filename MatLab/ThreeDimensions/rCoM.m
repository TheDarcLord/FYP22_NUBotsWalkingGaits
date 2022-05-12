function [r0CoM] = rCoM(q,params)
% rCOM  [3D Model] Centre of Mass - CoM
%       Relies on the FKM, k(q), to locate the masses
%       r0CoM - Position[x,y,z] of CoM in ZERO/World Coordinates
%       Equation:
%           M = Σ mᵢ
%           Ṟ = M⁻¹ Σ ṟᵢmᵢ
    %% JOINT VARIABLES
    q1  = q(1);     % θ₁
    q2  = q(2);     % θ₂
    q3  = q(3);     % θ₃
    q4  = q(4);     % θ₄
    q5  = q(5);     % θ₅
    q8  = q(8);     % θ₈
    q9  = q(9);     % θ₉
    q10 = q(10);    % θ₁₀
    q11 = q(11);    % θ₁₁
    q12 = q(12);    % θ₁₂

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

    %% Lengths
    Ll = params.fibula;     % Lower Leg
    Lu = params.femur;      % Upper Leg
    S  = params.ServoSize;  % SERVO DIST
   
    %% Masses
    mLl = params.mass.femur;    % Thigh Bone
    mLu = params.mass.fibula;   % Paired with `tibia`
    mJo = params.mass.joint;    % Knee Bone / Joints
    mPe = params.mass.pelvis;   % Waist Mass
    
    %% JOINT POSITIONS
    A0Sl   = [eye(3), params.r0Lg;  % LEFT  Sole Position from 
              zeros(1,3),       1]; %       0rigin in Global
    A0Sr   = [eye(3), params.r0Rg;  % RIGHT Sole Position from 
              zeros(1,3),       1]; %       0rigin in Global
    A0H    = [eye(3), params.r0Hg;  % PELVIS Mid  Position from 
              zeros(1,3),       1]; %       0rigin in Global
    
%% LEFT
    % LEFT ANKLE
    A01 = A0Sl * AE1;
        A12x = -S*sin(q1);
        A12y =  S*cos(q1);
        A12z =  0;
        A12R = [0, -sin(q1), -cos(q1);
                0,  cos(q1), -sin(q1);
                1,        0,        0];
        A12  = [A12R, [A12x A12y A12z]'; ROW4];
    A0Al = A01 * A12;
    % LEFT KNEE: A0Kl = A02 * A23
        A23x = -Ll*sin(q2);
        A23y =  Ll*cos(q2);
        A23z =  0;
        A23R = [cos(q2), -sin(q2), 0;
                sin(q2),  cos(q2), 0;
                      0,        0, 1];
        A23  = [A23R, [A23x A23y A23z]'; ROW4];
    A0Kl = A0Al * A23;
    % LEFT HIP: A0Hl = A0Kl * A34 * A45 * A56
        A34x = -Lu*sin(q3);
        A34y =  Lu*cos(q3);
        A34z =  0;
        A34R = [cos(q3), -sin(q3), 0;
                sin(q3),  cos(q3), 0;
                      0,        0, 1];
        A34  = [A34R, [A34x A34y A34z]'; ROW4];
    A04 = A0Kl * A34;
        A45x = -S*sin(q4);
        A45y =  S*cos(q4);
        A45z =  0;
        A45R = [0, -sin(q4),  cos(q4);
                0,  cos(q4),  sin(q4);
               -1,        0,        0];
        A45  = [A45R, [A45x A45y A45z]'; ROW4];
    A05 = A04 * A45;
        A56x = -S*sin(q5);
        A56y =  S*cos(q5);
        A56z =  0;
        A56R = [cos(q5), 0,  sin(q5);
                sin(q5), 0, -cos(q5);
                      0, 1,        0];
        A56  = [A56R, [A56x A56y A56z]'; ROW4];
    A0Hl = A05 * A56;
%% RIGHT
    % RIGHT ANKLE
    A012 = A0Sr * AE12;
        A1211x =  S*sin(q12);
        A1211y =  S*cos(q12);
        A1211z =  0;
        A1211R = [0, sin(q12), -cos(q12);
                  0, cos(q12),  sin(q12);
                  1,        0,         0];
        A1211  = [A1211R, [A1211x A1211y A1211z]'; ROW4];
    A0Ar = A012 * A1211;
    % RIGHT KNEE: A0Kr = A011 * A1110
        A1110x =  Ll*sin(q11);
        A1110y =  Ll*cos(q11);
        A1110z =  0;
        A1110R = [cos(q11), sin(q11), 0;
                 -sin(q11), cos(q11), 0;
                         0,        0, 1];
        A1110  = [A1110R, [A1110x A1110y A1110z]'; ROW4];
    A0Kr = A0Ar * A1110;
    
    % RIGHT HIP:  A0Hr = A0Kr * A109 * A98 * A87
        A109x =  Lu*sin(q10);
        A109y =  Lu*cos(q10);
        A109z =  0;
        A109R = [cos(q10), sin(q10), 0;
                -sin(q10), cos(q10), 0;
                       0,       0,   1];
        A109  = [A109R, [A109x A109y A109z]'; ROW4];
    A09  = A0Kr * A109;
        A98x = S*sin(q9);
        A98y = S*cos(q9);
        A98z = 0;
        A98R = [0, sin(q9),  cos(q9);
                0, cos(q9), -sin(q9);
               -1,       0,       0];
        A98  = [A98R, [A98x A98y A98z]'; ROW4];
    A08  = A09 * A98;
        A87x =  S*sin(q8);
        A87y =  S*cos(q8);
        A87z =  0;
        A87R = [cos(q8), 0, -sin(q8);
               -sin(q8), 0, -cos(q8);
                      0, 1,        0];
        A87  = [A87R, [A87x A87y A87z]'; ROW4];
    A0Hr = A08 * A87;
    
    %% LINK POSITIONS
    A0Ul = (A0Hl(1:3,4) + A0Kl(1:3,4)) * 0.5; % Estimate of rCoM of UL L
    A0Ll = (A0Kl(1:3,4) + A0Al(1:3,4)) * 0.5; % Estimate of rCoM of LL L
    A0Ur = (A0Hr(1:3,4) + A0Kr(1:3,4)) * 0.5; % Estimate of rCoM of UL R
    A0Lr = (A0Kr(1:3,4) + A0Ar(1:3,4)) * 0.5; % Estimate of rCoM of LL R

    %% Position of the CoM in Global Coordinates
    sigmaMass = (mJo * 14) + ... Joints (Sole, Ankle, Knee, Hip, Servos)
                (mLu * 2)  + ... Upper Leg
                (mLl * 2)  + ... Lower Leg
                 mPe;
    r0CoM = ((A0Sl(1:3,4) + A0Al(1:3,4) + A0Kl(1:3,4) + A0Hl(1:3,4)) * mJo + ...
             (A0Sr(1:3,4) + A0Ar(1:3,4) + A0Kr(1:3,4) + A0Hr(1:3,4)) * mJo + ...
             (A01(1:3,4)  + A04(1:3,4)  + A05(1:3,4)) * mJo + ...
             (A012(1:3,4) + A09(1:3,4)  + A08(1:3,4)) * mJo + ...
             (A0H(1:3,4))  * mPe + ...
             (A0Ul + A0Ur) * mLu + ...
             (A0Ll + A0Lr) * mLl ) ...
             / sigmaMass;
end

