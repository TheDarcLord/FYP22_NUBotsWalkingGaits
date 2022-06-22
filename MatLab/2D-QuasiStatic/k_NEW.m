function [xe, HTs] = k_NEW(q, index, model, params)
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

    %% HELPER Functions
    Rz = @(psi) [cos(psi),-sin(psi),0,0;
                 sin(psi), cos(psi),0,0;
                        0,        0,1,0;
                        0,        0,0,1];
    T  = @(x,y,z) [1,0,0,x;
                   0,1,0,y;
                   0,0,1,z;
                   0,0,0,1];

    T06 = Rz(q(1)) *T(0, Ll, 0) ... % A01(θ₁)
        * Rz(q(2)) *T(0, Lu, 0) ... % A12(θ₂)
        * Rz(q(3)) *T(0,  0,-H) ... % A23(θ₃)
        * Rz(q(4)) *T(0,-Lu, 0) ... % A34(θ₄)
        * Rz(q(5)) *T(0,-Ll, 0) ... % A45(θ₅)
        * Rz(q(6));                 % A56(θ₆)
    

    %% HOMOGENOUS TRANSFORM
    if params.mode == -1    % LEFT FIXED
        Tb0 = T(rBLb(1),rBLb(2),rBLb(3)) *T(0,Sa,0);
        Tne = T(0,-Sa, 0);
        Tbe = Tb0*T06*Tne;
    elseif params.mode == 0     % BOTH FIXED
        TbL  = T(rBLb(1),rBLb(2),rBLb(3)) *T(0,Sa,0);
        T02e = Rz(q(1)) *T(0, Ll,    0)  ... % A01(θ₁)
             * Rz(q(2)) *T(0, Lu,    0)  ... % A12(θ₂)
             * Rz(q(3)) *T(0,  0,-(H/2));    % A2e(θ₃)
        TbeL = TbL * T02e;

        TbR  = T(rBRb(1),rBRb(2),rBRb(3)) *T(0,Sa,0);
        Te36 = T(0,  0,(-H/2))       ... % Ae3(θ₃)
             * Rz(q(4)) *T(0,-Lu, 0) ... % A34(θ₄)
             * Rz(q(5)) *T(0,-Ll, 0) ... % A45(θ₅)
             * Rz(q(6));                 % A56(θ₆)
        T63e = Te36\eye(4);
        TbeR = TbR * T63e;
    elseif params.mode == 1     % RIGHT FIXED
        Tb0 = T(rBRb(1),rBRb(2),rBRb(3)) *T(0,Sa,0);
        Tne = T(0,-Sa, 0);
        Tbe = Tb0*(T06\eye(4))*Tne;
    end

    %% Homogeneous Transforms
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
        HTs.Ab3 = HTs.Ab2 * Rz(q(3)) *T(0,  0,(-H/2));

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