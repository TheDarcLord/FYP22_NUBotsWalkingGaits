function [r0CoM] = rCoM(q,index,model,params)
% rCOM  [2D Model] Centre of Mass - CoM
%       Relies on the FKM, k(q), to locate the masses
%       r0CoM - Position[x,y,z] of CoM in ZERO/World Coordinates
%       Equation:
%           M = Σ mᵢ
%           Ṟ = M⁻¹ Σ ṟᵢmᵢ
    %% JOINT VARIABLES
    sigT12      = sum(q(1:2));   % Σθᵢ    i=1:2
    sigT56      = sum(q(5:6));   % Σθᵢ    i=5:6

    %% Lengths
    Ll = params.fibula;     % Lower Leg
    Lu = params.femur;      % Upper Leg
    h  = params.HipWidth;   % Pelvis Width
   
    %% Masses
    mLl = params.mass.femur;    % Thigh Bone
    mLu = params.mass.fibula;   % Paired with `tibia`
    mJo = params.mass.joint;    % Knee Bone / Joints
    mPe = params.mass.pelvis;   % Waist Mass
    
    %% JOINT POSITIONS
    A0Al   = [eye(3), model.rBLb(:,index);  % LEFT Ankle Position from 
              0,0,0,                   1]; %         0rigin in Global
    A0Ar   = [eye(3), model.rBRb(:,index);  % RIGHT Ankle Position from 
              0,0,0,                   1]; %         0rigin in Global
    A0H    = [eye(3), model.rBHb(:,index);  % PELVIS Mid  Position from 
              0,0,0,                   1]; %         0rigin in Global
    
    % LEFT KNEE = A0L * A12
    A12x = -Ll*sin(q(1));
    A12y =  Ll*cos(q(1));
    A12z =  0;
    A0Kl = A0Al*[cos( q(1) ), -sin( q(1) ), 0, A12x;
                 sin( q(1) ),  cos( q(1) ), 0, A12y;
                           0,            0, 1, A12z;
                           0,            0, 0,    1];
    % LEFT HIP = A0L * A13
    A13x = -Lu*sin(sigT12)-Ll*sin(q(1));
    A13y =  Lu*cos(sigT12)+Ll*cos(q(1));
    A13z =  0;
    A0Hl = A0Al*[cos(sigT12), -sin(sigT12), 0, A13x;
                 sin(sigT12),  cos(sigT12), 0, A13y;
                           0,            0, 1, A13z;
                           0,            0, 0,    1];
    % RIGHT HIP = A0R * A64
    A64x = Lu*sin(sigT56)+Ll*sin(q(6));
    A64y = Lu*cos(sigT56)+Ll*cos(q(6));
    A64z = 0;
    A0Hr = A0Ar*[cos(sigT56), sin(sigT56), 0, A64x;
                -sin(sigT56), cos(sigT56), 0, A64y;
                           0,           0, 1, A64z;
                           0,           0, 0,    1];
    % RIGHT KNEE = A0R * A65
    A65x = Ll*sin(q(6));
    A65y = Ll*cos(q(6));
    A65z = 0;
    A0Kr = A0Ar*[cos( q(6) ), sin( q(6) ), 0, A65x;
                -sin( q(6) ), cos( q(6) ), 0, A65y;
                           0,           0, 1, A65z;
                           0,           0, 0,    1];

    %% LINK POSITIONS
    A0Ul = (A0Hl(1:3,4) + A0Kl(1:3,4)) * 0.5; % Estimate of rCoM of UL L
    A0Ll = (A0Kl(1:3,4) + A0Al(1:3,4)) * 0.5; % Estimate of rCoM of LL L
    A0Ur = (A0Hr(1:3,4) + A0Kr(1:3,4)) * 0.5; % Estimate of rCoM of UL R
    A0Lr = (A0Kr(1:3,4) + A0Ar(1:3,4)) * 0.5; % Estimate of rCoM of LL R

    %% Position of the CoM in Global Coordinates
    sigmaMass = (mJo * 6) + ... Joints (Ankle, Knee, Hip)
                (mLu * 2) + ... Upper Leg
                (mLl * 2) + ... Lower Leg
                 mPe;
    r0CoM = ((A0Al(1:3,4) + A0Kl(1:3,4) + A0Hl(1:3,4)) * mJo + ...
             (A0Ar(1:3,4) + A0Kr(1:3,4) + A0Hr(1:3,4)) * mJo + ...
             (A0H(1:3,4))  * mPe + ...
             (A0Ul + A0Ur) * mLu + ...
             (A0Ll + A0Lr) * mLl ) ...
             / sigmaMass;
end

