function [HTs] = kSLOW(q, index, model, params)
% k(q)  [2D Model] Forward Kinematic Model - FKM
%       
%       Returns:    [xe, TAA, Transforms] for an array of 'q'
%       xe:         End Effector Pose [X Y Z ϕ θ Ψ]'
%       TAA:        Transform from ANKLE to END EFFECTOR
%       Transforms: All other Homogenous Transforms
    
    %% HELPER FUNCTIONS
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

    ABEL    = [Rzyx(model.r0Lg(6,index), ...
                model.r0Lg(5,index), ...
                model.r0Lg(4,index)),... 
                model.r0Lg(1:3,index);  % LEFT Ankle Position from 
                zeros(1,3),           1]; %           0rigin in Global

    ABER    = [Rzyx(model.r0Rg(6,index), ...
                    model.r0Rg(5,index), ...
                    model.r0Rg(4,index)),... 
                    model.r0Rg(1:3,index);  % RIGHT Ankle Position from 
                zeros(1,3),           1]; %           0rigin in Global

    %% HOMOGENOUS TRANSFORM
    % TB_0 * [ A⁰₁(q₁)⋅A¹₂(q₂) ... Aᴶ⁻¹ⱼ  ] * T12_B

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

    %% EXPORT
    HTs.ABEL = ABEL;
    HTs.ALB0 = HTs.ABEL*TB0;
    HTs.A01  = HTs.ALB0*A01;
    HTs.A02  = HTs.A01 *A12;
    HTs.A03  = HTs.A02 *A23;
    HTs.A04  = HTs.A03 *A34;
    HTs.A05  = HTs.A04 *A45;
    HTs.A06  = HTs.A05 *A56;
     
%     HTs.A07  = HTs.A06 *A67;
%     HTs.A08  = HTs.A07 *A78;
%     HTs.A09  = HTs.A08 *A89;
%     HTs.A010 = HTs.A09 *A910;
%     HTs.A011 = HTs.A010*A1011;
%     HTs.ARB0 = HTs.A011*A1112;
%     HTs.ABER = HTs.ARB0*T12B;

    HTs.ABER = ABER;
    HTs.ARB0 = HTs.ABER * inv(T12B);
    HTs.A011 = HTs.ARB0 * inv(A1112);
    HTs.A010 = HTs.A011 * inv(A1011);
    HTs.A09  = HTs.A010 * inv(A910);
    HTs.A08  = HTs.A09  * inv(A89);
    HTs.A07  = HTs.A08  * inv(A78);

%     HTs.A06  = HTs.A07  * inv(A67);
%     HTs.A05  = HTs.A06  * inv(A56);
%     HTs.A04  = HTs.A05  * inv(A45);
%     HTs.A03  = HTs.A04  * inv(A34);
%     HTs.A02  = HTs.A03  * inv(A23);
%     HTs.A01  = HTs.A02  * inv(A12);
%     HTs.ALB0 = HTs.A01  * inv(A01);
%     HTs.ABEL = HTs.ALB0 * inv(TB0);
end