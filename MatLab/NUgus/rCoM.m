function [r0CoM] = rCoM(q, params)
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
    q6  = q(6);     % θ₆
    q7  = q(7);     % θ₇
    q8  = q(8);     % θ₈
    q9  = q(9);     % θ₉
    q10 = q(10);    % θ₁₀
    q11 = q(11);    % θ₁₁
    q12 = q(12);    % θ₁₂

    %% Lengths
    H    = params.HipWidth;
    h2a  = params.heel2ankle;
    a2k  = params.ankle2knee;
    k2h  = params.knee2hip;
    h2w  = params.hip2waist;
   
    %% Masses
    mFo = params.mass.foot;     % Foot
    mLl = params.mass.femur;    % Upper Leg
    mLu = params.mass.fibula;   % Lower Leg
    mJo = params.mass.joint;    % Joints
    mPe = params.mass.pelvis;   % Waist Mass

    TB0   = [0,0, 1,0; 0,1,0, h2a;
            -1,0, 0,0; 0,0,0,  1];
    % INVERTIBLE !!!
    A01   = [0, -sin(q1), -cos(q1), 0;
             0,  cos(q1), -sin(q1), 0;
             1,        0,        0, 0;
             0,        0,        0, 1];
    A12   = [cos(q2), -sin(q2),  0, a2k(1)*cos(q2)-a2k(2)*sin(q2);
             sin(q2),  cos(q2),  0, a2k(2)*cos(q2)+a2k(1)*sin(q2);
                   0,        0,  1,                        a2k(3);
                   0,        0,  0,                            1];
    A23   = [cos(q3), -sin(q3),  0, k2h(1)*cos(q3)-k2h(2)*sin(q3);
             sin(q3),  cos(q3),  0, k2h(2)*cos(q3)+k2h(1)*sin(q3);
                   0,        0,  1,                        k2h(3);
                   0,        0,  0,                            1];
    A34   = [ 0, -sin(q4), cos(q4), h2w(1)*cos(q4);
              0,  cos(q4), sin(q4), h2w(1)*sin(q4);
             -1,        0,       0,              0;
              0,        0,       0,             1];
    A45   = [cos(q5),  0,  sin(q5),  h2w(3)*sin(q5);
             sin(q5),  0, -cos(q5), -h2w(3)*cos(q5);
                   0,  1,        0,          h2w(2);
                   0,  0,        0,              1];
    A56   = [cos(q6), -sin(q6),  0, H*cos(q6);
             sin(q6),  cos(q6),  0, H*sin(q6);
                   0,        0,  1,         0;
                   0,        0,  0,         1];
    A67   = [cos(q7),  0, -sin(q7),  h2w(2)*sin(q7);
             sin(q7),  0,  cos(q7), -h2w(2)*cos(q7);
                   0, -1,        0,         -h2w(3);
                   0,  0,        0,              1];
    A78   = [0, -sin(q8), -cos(q8),     0;
             0,  cos(q8), -sin(q8),     0;
             1,        0,        0, -h2w(1);
             0,        0,        0,     1];
    A89   = [cos(q9), -sin(q9),  0,  k2h(2)*sin(q9)-k2h(1)*cos(q9);
             sin(q9),  cos(q9),  0, -k2h(2)*cos(q9)-k2h(1)*sin(q9);
                   0,        0,  1,                        -k2h(3);
                   0,        0,  0,                             1];
    A910  = [cos(q10), -sin(q10),  0,  a2k(2)*sin(q10)-a2k(1)*cos(q10);
             sin(q10),  cos(q10),  0, -a2k(2)*cos(q10)-a2k(1)*sin(q10);
                    0,         0,  1,                          -a2k(3);
                    0,         0,  0,                               1];
    A1011 = [ 0, -sin(q11), cos(q11), 0;
              0,  cos(q11), sin(q11), 0;
             -1,         0,        0, 0;
              0,         0,        0, 1];
    A1112 = [cos(q12), -sin(q12),  0, 0;
             sin(q12),  cos(q12),  0, 0;
                    0,         0,  1, 0;
                    0,         0,  0, 1];
    % INVERTIBLE !!!
    T12B  = [0,0,-1,0; 0,1,0,-h2a;
             1,0, 0,0; 0,0,0,  1];

    %% TRANSFORMS
    if params.mode ==  1        % RIGHT FIXED
        AB0 = TB0;
        AB1  = AB0*A01;
        AB2  = AB1 *A12;
        AB3  = AB2 *A23;
        AB4  = AB3 *A34;
        AB5  = AB4 *A45;
        AB6  = AB5 *A56;
         
        AB7  = AB6 *A67;
        AB8  = AB7 *A78;
        AB9  = AB8 *A89;
        AB10 = AB9 *A910;
        AB11 = AB10*A1011;
        AB12 = AB11*A1112;
        ABE  = AB12*T12B;
    elseif params.mode == -1     % LEFT FIXED
        AE12 = inv(T12B);
        AB11 = AE12 * inv(A1112);
        AB10 = AB11 * inv(A1011);
        AB9  = AB10 * inv(A910);
        AB8  = AB9  * inv(A89);
        AB7  = AB8  * inv(A78);
    
        AB6  = AB7  * inv(A67);
        AB5  = AB6  * inv(A56);
        AB4  = AB5  * inv(A45);
        AB3  = AB4  * inv(A34);
        AB2  = AB3  * inv(A23);
        AB1  = AB2  * inv(A12);
        AB0  = AB1  * inv(A01);
        ABE  = AB0  * inv(TB0);
    end

    %% Position of the CoM in Global Coordinates
    sigmaMass = (mFo * 2)  + ... Foot
                (mJo * 12) + ... Joints (Ankle, Knee, Hip, Servos)
                (mLu * 2)  + ... Upper Leg
                (mLl * 2)  + ... Lower Leg
                 mPe;
%     r0CoM = (([0;0;0]   + ABE(1:3,4)) *mFo + ... FEET
%             (AB0(1:3,4) + AB11(1:3,4)) *mJo + ...
%             (AB1(1:3,4) + AB10(1:3,4)) *mJo + ...
%             (AB2(1:3,4) + AB9(1:3,4)) *mJo + ...
%             (AB3(1:3,4) + AB8(1:3,4)) *mJo + ...
%             (AB4(1:3,4) + AB7(1:3,4)) *mJo + ...
%             (AB5(1:3,4) + AB6(1:3,4)) *mJo + ...
%             (AB5(1:3,4) + AB6(1:3,4)) *(mPe/2) + ...
%             (AB1(1:3,4) + AB2(1:3,4)) *(mLl/2) + ...
%             (AB10(1:3,4) + AB9(1:3,4)) *(mLl/2) + ...
%             (AB2(1:3,4) + AB3(1:3,4)) *(mLu/2) + ...
%             (AB9(1:3,4) + AB8(1:3,4)) *(mLu/2) ...
%             )/sigmaMass;

    r0CoM = (AB5(1:3,4) + AB6(1:3,4))/2;
end

