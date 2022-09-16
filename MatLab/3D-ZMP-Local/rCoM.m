function [r0CoM] = rCoM(q, index, model, params)
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

    Rzyx   = @(Rz,Ry,Rx) ...
        [ cos(Rz)*cos(Ry), -sin(Rz)*cos(Rx)+cos(Rz)*sin(Ry)*sin(Rx),...
                    sin(Rz)*sin(Rx)+cos(Rz)*sin(Ry)*cos(Rx);
          sin(Rz)*cos(Ry),  cos(Rz)*cos(Rx)+sin(Rz)*sin(Ry)*sin(Rx),...
                   -cos(Rz)*sin(Rx)+sin(Rz)*sin(Ry)*cos(Rx);
                 -sin(Ry),  cos(Ry)*sin(Rx)                        ,...
                    cos(Ry)*cos(Rx)];

    %% Lengths
    Ll = params.fibula;     % Lower Leg
    Lu = params.femur;      % Upper Leg
    S  = params.ServoSize;  % SERVO DIST
   
    %% Masses
    mLl = params.mass.femur;    % Thigh Bone
    mLu = params.mass.fibula;   % Paired with `tibia`
    mJo = params.mass.joint;    % Knee Bone / Joints
    mPe = params.mass.pelvis;   % Waist Mass

    %% Position of the CoM in Global Coordinates
    sigmaMass = (mJo * 14) + ... Joints (Sole, Ankle, Knee, Hip, Servos)
                (mLu * 2)  + ... Upper Leg
                (mLl * 2)  + ... Lower Leg
                 mPe;
    r0CoM = [0;0;0] / sigmaMass;
end

