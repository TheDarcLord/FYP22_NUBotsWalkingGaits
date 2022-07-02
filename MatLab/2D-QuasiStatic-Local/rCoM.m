function [rCoMb] = rCoM(q,index,model,params)
% rCOM  [2D Model] Centre of Mass - CoM
%       Relies on the FKM, k(q), to locate the masses
%       r0CoM - Position[x,y,z] of CoM in ZERO/World Coordinates
%       Equation:
%           M = Σ mᵢ
%           Ṟ = M⁻¹ Σ ṟᵢmᵢ

    %% Lengths
    Ll = params.fibula;     % Lower Leg
    Lu = params.femur;      % Upper Leg
    H  = params.HipWidth;   % Pelvis Width
    Sa = params.tarsal;     % Sole to Ankle
    % RATIO
    R  = 0.5;
    %% Masses
    mLu = params.mass.femur;    % Thigh Bone Mass
    mLl = params.mass.fibula;   % Paired with `tibia` Mass
    mJo = params.mass.joint;    % Knee Bone / Joints Mass
    mPe = params.mass.pelvis;   % Waist Mass
    mFe = params.mass.foot;     % Foot Mass

    %% JOINT & LINK Mass Position
    rBLb = model.rBLb(:,index);                 % LEFT  Sole
        rB0b  = [rBLb(1); rBLb(2); -Sa+rBLb(3)]; %       Ankle
        rFoLb = R*(rBLb+rB0b);                  %       Foot
        rB1b  = [rB0b(1)+Ll*sin(q(1));          %       Knee
                 rB0b(2);
                 rB0b(3)-Ll*cos(q(1))];
        rLlLb = R*(rB0b+rB1b);                  %       Fibula
        rB2b  = [rB1b(1)+Lu*sin(q(1)+q(2)); 
                 rB1b(2); 
                 rB1b(3)-Lu*cos(q(1)+q(2))];    %       Hip
        rLuLb = R*(rB1b+rB2b);                  %       Femur 
    rBRb = model.rBRb(:,index);                 % RIGHT Sole
        rB5b  = [rBRb(1); rBRb(2); -Sa+rBRb(3)]; %       Ankle
        rFoRb = R*(rBRb+rB5b);                  %       Foot
        rB4b  = [rB5b(1)+Ll*sin(q(6));          %       Knee
                 rB5b(2);
                 rB5b(3)-Ll*cos(q(6))];
        rLlRb = R*(rB5b+rB4b);                  %       Fibula
        rB3b  = [rB4b(1)+Lu*sin(q(5)+q(6));
                 rB4b(2);
                 rB4b(3)-Lu*cos(q(5)+q(6))];    %       Hip
        rLuRb = R*(rB4b+rB3b);                  %       Femur
    rBHb = [rB3b(1);
            rB3b(2)+(H/2);
            rB3b(3)];

    %% Position of the CoM in Base Co-ordinates
    sigmaMass = (mJo * 6) + ...% Joints (Ankle, Knee, Hip)
                (mFe * 2) + ...% Feet
                (mLl * 2) + ...% Upper Legs
                (mLu * 2) + ...% Lower Legs
                 mPe;          % Pelvis
    rCoMb = ( ...
        + (mPe*rBHb) ...
        + (mJo*(rB5b+rB4b+rB3b+rB2b+rB1b+rB0b)) ...
        + (mFe*(rFoLb+rFoRb)) ...
        + (mLl*(rLlLb+rLlRb)) ...
        + (mLu*(rLuLb+rLuRb)) ...
    )/ sigmaMass;
end

