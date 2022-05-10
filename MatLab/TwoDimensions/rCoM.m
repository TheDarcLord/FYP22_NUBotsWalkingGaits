function [r0CoM] = rCoM(HomogeneousTransforms,params)
% rCOM  [2D Model] Centre of Mass - CoM
%       Relies on the FKM, k(q), to locate the masses
%       r0CoM - Position[x,y,z] of CoM in ZERO/World Coordinates
%       Equation:
%           M = Σ mᵢ
%           Ṟ = M⁻¹ Σ ṟᵢmᵢ
    HT = HomogeneousTransforms;
    
    % Lengths
    lf = params.fibula;
    uf = params.femur;
    h  = params.HipWidth;
   
    % Masses
    ufm = params.mass.femur;    % Thigh Bone
    lfm = params.mass.fibula;   % Paired with `tibia`
    jm  = params.mass.joint;    % Knee Bone / Joints
    wm  = params.mass.pelvis;   % Waist Mass
    
    % Joints
    j1m = jm*HT.A01(1:3,4);
    j2m = jm*HT.A02(1:3,4);
    j3m = jm*HT.A03(1:3,4);
    j4m = jm*HT.A04(1:3,4);
    j5m = jm*HT.A05(1:3,4);
    j6m = jm*HT.A06(1:3,4);

    % Links                     X Y Z
    r1m = lfm*(HT.A01(1:3,4) + [0;  lf/2;    0]); % Fibula / Lower Leg
    r2m = ufm*(HT.A02(1:3,4) + [0;  uf/2;    0]); % Femur  / Upper Leg
    r3m = wm *(HT.A03(1:3,4) + [0;     0; -h/2]); % Mid Waist
    r4m = ufm*(HT.A04(1:3,4) + [0; -uf/2;    0]); % Femur  / Upper Leg
    r5m = lfm*(HT.A05(1:3,4) + [0; -lf/2;    0]); % Fibula / Lower Leg
    r6m = (HT.A06(1:3,4) + [0;0;0]); % Moving Foot ... 
    
    sigmaMass = 2*(lfm + ufm) + wm + 6*jm;
    
    r0CoM = ((j1m + j2m + j3m + j4m + j5m + j6m) + ... 
             (r1m + r2m + r3m + r4m + r5m)) / sigmaMass;
end

