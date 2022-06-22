function [xe, HTs] = k(q, index, model, params)
% k(q)  [2D Model] Forward Kinematic Model - FKM
%       
%       Returns:    [xe, TAA, Transforms] for an array of 'q'
%       xe:         End Effector Pose [X Y Z ϕ θ Ψ]'
%       TAA:        Transform from ANKLE to END EFFECTOR
%       Transforms: All other Homogenous Transforms

    %% LINK VARIABLES
    Ll     = params.fibula;     % Lower Leg
    Lu     = params.femur;      % Upper Leg
    Sa     = params.tarsal;     % Sole to Ankle
    H      = params.HipWidth;
    
    rBLb    = model.rBLb(:,index); % LEFT  Sole Position
    rBRb    = model.rBRb(:,index); % RIGHT Sole Position
    %% HOMOGENEOUS CONSTANTS
    TNE  = [1, 0, 0,   0; 
            0, 1, 0, -Sa;
            0, 0, 1,   0; 
            0, 0, 0,   1];
    Tb0R = [1, 0, 0, rBRb(1); 0, 1, 0, Sa+rBRb(2);
            0, 0, 1, rBRb(3); 0, 0, 0,         1];
    Tb0L = [1, 0, 0, rBLb(1); 0, 1, 0, Sa+rBLb(2);
            0, 0, 1, rBLb(3); 0, 0, 0,         1];

    %% q Variables
    sig16 = sum(q);
    sig15 = sum(q(1:5));
    sig14 = sum(q(1:4));
    sig13 = sum(q(1:3));
    sig12 = sum(q(1:2));
    sig46 = sum(q(4:6));
    sig56 = sum(q(5:6));
    sig36 = sum(q(3:6));
    sig26 = sum(q(2:6));

    %% STEP LOGIC
    if params.mode == -1    % LEFT FIXED
        r06 = [Lu*(sin(sig14)-sin(sig12))-Ll*(sin(q(1))-sin(sig15));
               Lu*(cos(sig12)-cos(sig14))+Ll*(cos(q(1))-cos(sig15));
              -H];
        R06 = [cos(sig16), -sin(sig16), 0;
               sin(sig16),  cos(sig16), 0;
                        0,           0, 1];
        T06 = [R06, r06;
               0,0,0,1];
        Tbe = Tb0L * T06 * TNE;
    elseif params.mode == 0 % BOTH FIXED
        T02e = [cos(sig13), -sin(sig13), 0, -Lu*sin(sig12)-Ll*sin(q(1));
                sin(sig13),  cos(sig13), 0,  Lu*cos(sig12)+Ll*cos(q(1));
                         0,           0, 1,                        -H/2;
                         0,           0, 0,                          1];
        T63e = [cos(sig46),  sin(sig46), 0,  Lu*sin(sig56)+Ll*sin(q(6));
               -sin(sig46),  cos(sig46), 0,  Lu*cos(sig56)+Ll*cos(q(6));
                         0,           0, 1,                         H/2;
                         0,           0, 0,                          1];
        TbeL = Tb0L*T02e;
        TbeR = Tb0R*T63e;

    elseif params.mode == 1 % RIGHT FIXED
        R60 = [cos(sig16),  sin(sig16), 0;
              -sin(sig16),  cos(sig16), 0;
                        0,           0, 1];
        r60 = [Lu*(sin(sig56)-sin(sig36))+Ll*(sin(q(6))-sin(sig26));
               Lu*(cos(sig56)-cos(sig36))+Ll*(cos(q(6))-cos(sig26));
               H];
        T60 = [R60, r60;
               0,0,0,1];
        
        Tbe = Tb0R * T60 * TNE;
    end

    %% Homogeneous Transforms
    % HELPER Functions
    Rz = @(psi) [cos(psi),-sin(psi),0,0;
                 sin(psi), cos(psi),0,0;
                        0,        0,1,0;
                        0,        0,0,1];
    T  = @(x,y,z) [1,0,0,x;
                   0,1,0,y;
                   0,0,1,z;
                   0,0,0,1];

    if params.mode == -1
        HTs.AbB = T(rBLb(1),rBLb(2),rBLb(3));
        HTs.Ab0 = HTs.AbB * T(0, Sa, 0);
        HTs.Ab1 = HTs.Ab0 * Rz(q(1)) *T(0, Ll, 0);
        HTs.Ab2 = HTs.Ab1 * Rz(q(2)) *T(0, Lu, 0);
        HTs.Ab3 = HTs.Ab2 * Rz(q(3)) *T(0,  0,-H);
        HTs.Ab4 = HTs.Ab3 * Rz(q(4)) *T(0,-Lu, 0);
        HTs.Ab5 = HTs.Ab4 * Rz(q(5)) *T(0,-Ll, 0);
        HTs.Ab6 = HTs.Ab5 * Rz(q(6));
        HTs.AbE = Tbe;

        HTs.ABRb = HTs.AbE;
        HTs.ABLb = HTs.AbB;
        HTs.AbH = HTs.Ab2 * T(0,  0, -(H/2));

    elseif params.mode == 0
        HTs.AbB = T(rBLb(1),rBLb(2),rBLb(3));
        HTs.Ab0 = HTs.AbB * T(0,Sa,0);
        HTs.Ab1 = HTs.Ab0 * Rz(q(1)) *T(0, Ll, 0);
        HTs.Ab2 = HTs.Ab1 * Rz(q(2)) *T(0, Lu, 0);
        HTs.Ab3 = HTs.Ab2 * Rz(q(3)) *T(0,  0,-H);

        HTs.AbE = T(rBRb(1),rBRb(2),rBRb(3));
        HTs.Ab6 = HTs.AbE * T(0,Sa,0);
        HTs.Ab5 = HTs.Ab6 * Rz(-q(6)) *T(0,Ll, 0);  
        HTs.Ab4 = HTs.Ab5 * Rz(-q(5)) *T(0,Lu, 0);
        HTs.AbH = HTs.Ab4 * Rz(-q(4)) *T(0,  0,(H/2));
        
        HTs.ABRb = HTs.AbE;
        HTs.ABLb = HTs.AbB;

    elseif params.mode == 1
        HTs.AbB = T(rBRb(1),rBRb(2),rBRb(3));
        HTs.Ab0 = HTs.AbB * T(0, Sa, 0);

        HTs.Ab1 = HTs.Ab0 * Rz(-q(6)) *T(0, Ll, 0); 
        HTs.Ab2 = HTs.Ab1 * Rz(-q(5)) *T(0, Lu, 0);
        HTs.Ab3 = HTs.Ab2 * Rz(-q(4)) *T(0,  0, H);
        HTs.Ab4 = HTs.Ab3 * Rz(-q(3)) *T(0,-Lu, 0);
        HTs.Ab5 = HTs.Ab4 * Rz(-q(2)) *T(0,-Ll, 0);
        HTs.Ab6 = HTs.Ab5 * Rz(-q(1));
        HTs.AbE = Tbe;

        HTs.ABRb = HTs.AbB;
        HTs.ABLb = HTs.AbE;
        HTs.AbH = HTs.Ab2 * T(0,  0, (H/2));
    end

    %% End Effector Parameterisation
    % R₁₁:  cθ₁cθ₂      Ψ = atan2( R₃₂, R₃₃)
    % R₂₁:  sθ₁cθ₂      θ = atan2(-R₃₁, SQRT(R₃₂² + R₃₃²))
    % R₃₁: -sθ₂         ϕ = atan2( R₂₁, R₁₁)
    % R₃₂:  0
    % R₃₃:  cθ₂
    
    if params.mode ~= 0
        phi =   atan2( Tbe(3,2), Tbe(3,3) );  
        theta = atan2(-Tbe(3,1), sqrt( Tbe(3,2)^2 + Tbe(3,3)^2 ) );
        psi =   atan2( Tbe(2,1), Tbe(1,1) );
        
        xe = [Tbe(1:3,4); % X Y Z
              phi;        % ϕ
              theta;      % θ
              psi];       % Ψ
    else
        Lphi   = 0;%atan2( TAEL(3,2), TAEL(3,3) );  
        Ltheta = 0;%atan2(-TAEL(3,1), sqrt( TAEL(3,2)^2 + TAEL(3,3)^2 ) );
        Lpsi   = 0;%atan2( TAEL(2,1), TAEL(1,1) );
        Rphi   = 0;%atan2( TAER(3,2), TAER(3,3) );  
        Rtheta = 0;%atan2(-TAER(3,1), sqrt( TAER(3,2)^2 + TAER(3,3)^2 ) );
        Rpsi   = 0;%atan2( TAER(2,1), TAER(1,1) );
        
        Lxe    = [TbeL(1:3,4);Lphi;Ltheta;Lpsi];
        Rxe    = [TbeR(1:3,4);Rphi;Rtheta;Rpsi];
        xe     = Lxe - Rxe;
    end
end