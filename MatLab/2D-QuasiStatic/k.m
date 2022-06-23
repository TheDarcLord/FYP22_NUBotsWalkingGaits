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
        T06 = [R06, r06; 0,0,0,1];
        Tbe = Tb0L * T06 * TNE;

        HTs.ABRb = Tbe;
        HTs.ABLb = [eye(3),rBRb; 0,0,0,1];
        HTs.AbH  = HTs.ABLb * ...
            [cos(sig13), -sin(sig13), 0, -Lu*sin(sig12)-Ll*sin(q(1));
             sin(sig13),  cos(sig13), 0,  Sa+Lu*cos(sig12)+Ll*cos(q(1));
                      0,           0, 1, -H/2;
                      0,           0, 0,  1];
    elseif params.mode == 0 % BOTH FIXED
        T02e = [cos(sig13), -sin(sig13), 0, -Lu*sin(sig12)-Ll*sin(q(1));
                sin(sig13),  cos(sig13), 0,  Lu*cos(sig12)+Ll*cos(q(1));
                         0,           0, 1,                        -H/2;
                         0,           0, 0,                          1];
        TbeL = Tb0L*T02e;
        T63e = [cos(sig46),  sin(sig46), 0,  Lu*sin(sig56)+Ll*sin(q(6));
               -sin(sig46),  cos(sig46), 0,  Lu*cos(sig56)+Ll*cos(q(6));
                         0,           0, 1,                         H/2;
                         0,           0, 0,                          1];
        TbeR = Tb0R*T63e;

        HTs.AbH  = TbeL;
        HTs.ABLb = [eye(3),rBRb; 0,0,0,1];
        HTs.ABRb = [eye(3),rBRb; 0,0,0,1];
    elseif params.mode == 1 % RIGHT FIXED
        R60 = [cos(sig16),  sin(sig16), 0;
              -sin(sig16),  cos(sig16), 0;
                        0,           0, 1];
        r60 = [Lu*(sin(sig56)-sin(sig36))+Ll*(sin(q(6))-sin(sig26));
               Lu*(cos(sig56)-cos(sig36))+Ll*(cos(q(6))-cos(sig26));
               H];
        T60 = [R60, r60; 0,0,0,1];
        Tbe = Tb0R * T60 * TNE;

        HTs.ABRb = [eye(3),rBRb; 0,0,0,1];
        HTs.ABLb = Tbe;
        HTs.AbH  = HTs.ABRb * ...
            [cos(sig46), sin(sig46), 0, Lu*sin(sig56)+Ll*sin(q(6));
            -sin(sig46), cos(sig46), 0, Sa+Lu*cos(sig56)+Ll*cos(q(6));
                      0,          0, 1, H/2;
                      0,          0, 0, 1];
    end

    %% End Effector Parameterisation
    % R₁₁:  cθ₁cθ₂      Ψ = atan2( R₃₂, R₃₃)
    % R₂₁:  sθ₁cθ₂      θ = atan2(-R₃₁, SQRT(R₃₂² + R₃₃²))
    % R₃₁: -sθ₂         ϕ = atan2( R₂₁, R₁₁)
    % R₃₂:  0
    % R₃₃:  cθ₂
    
    if params.mode ~= 0
        phi    = atan2( Tbe(3,2), Tbe(3,3) );  
        theta  = atan2(-Tbe(3,1), sqrt( Tbe(3,2)^2 + Tbe(3,3)^2 ) );
        psi    = atan2( Tbe(2,1), Tbe(1,1) );
        
        xe = [Tbe(1:3,4); % X Y Z
              phi;        % ϕ
              theta;      % θ
              psi];       % Ψ
    else
        Lphi   = 0;%atan2( TbeL(3,2), TbeL(3,3) );  
        Ltheta = 0;%atan2(-TbeL(3,1), sqrt( TbeL(3,2)^2 + TbeL(3,3)^2 ) );
        Lpsi   = atan2( TbeL(2,1), TbeL(1,1) );
        Rphi   = 0;%atan2( TbeR(3,2), TbeR(3,3) );  
        Rtheta = 0;%atan2(-TbeR(3,1), sqrt( TbeR(3,2)^2 + TbeR(3,3)^2 ) );
        Rpsi   = atan2( TbeR(2,1), TbeR(1,1) );
        
        Lxe    = [TbeL(1:3,4);Lphi;Ltheta;Lpsi];
        Rxe    = [TbeR(1:3,4);Rphi;Rtheta;Rpsi];
        xe     = Lxe - Rxe;
    end
end