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
    H      = params.HipWidth;   % Hip Width

    rGBg   = model.rGBg(:,index); % BODY  Location in Global Co-ordinates
    rBLb   = model.rBLb(:,index); % LEFT  Location in Body Co-ordinates
    rBRb   = model.rBRb(:,index); % RIGHT Location in Body Co-ordinates

    %% HELPER FUNCTIONS
    T  = @(x,y,z) [eye(3), [x,y,z]'; 0,0,0,1];
    Rz = @(psi)   [cos(psi), -sin(psi),0,0;
                   sin(psi),  cos(psi),0,0;
                          0,         0,1,0; 0,0,0,1];
    Ry = @(theta) [cos(theta), 0, sin(theta),0; 0,1,0,0;
                  -sin(theta), 0, cos(theta),0; 0,0,0,1];
    Rx = @(phi)   [1,          0,       0,0;
                   0, cos(phi), -sin(phi),0; 
                   0, sin(phi),  cos(phi),0; 
                   0,        0,         0,1];
    %% Origin
    TGBg = T(rGBg(1),rGBg(2),rGBg(3));
    TBRb = T(rBRb(1),rBRb(2),rBRb(3))* Rx(-pi/2);
    TBLb = T(rBLb(1),rBLb(2),rBLb(3))* Rx(-pi/2);

    TB0l = TGBg*Rx(-pi/2)*T(0,0,H/2);
    TB0r = TGBg*Rx(-pi/2)*T(0,0,-H/2);
    TNE  = Rx( pi/2);

    %% STEP LOGIC
    if params.mode == -1    % LEFT FIXED
        TBE = TBLb * inv( ...
               Rz(q(3))*T(0,-Lu,0) ... A01-L
               *Rz(q(2))*T(0,-Ll,0) ... A12-L
               *Rz(q(1))*T(0,-Sa,0) ... A23-L
            )* T(0,0,-H) ...
            *Rz(q(4))*T(0,-Lu,0) ... A01-R
            *Rz(q(5))*T(0,-Ll,0) ... A12-R
            *Rz(q(6))*T(0,-Sa,0) ... A23-R
            *TNE;  % RIGHT ANKLE
        
        TBF = TB0l ...
            *Rz(q(3))*T(0,-Lu,0) ... A01-L
            *Rz(q(2))*T(0,-Ll,0) ... A12-L
            *Rz(q(1))*T(0,-Sa,0) ... A23-L
            *TNE; % FIXED ANKLE

    elseif params.mode == 0 % BOTH FIXED
        TBEl = TBLb *inv( ...
                Rz(q(3))*T(0,-Lu,0) ... A01-L
               *Rz(q(2))*T(0,-Ll,0) ... A12-L
               *Rz(q(1))*T(0,-Sa,0) ... A23-L
            )* T(0,0,-H/2) * TNE;
        TBEr = TBRb *inv( ...
                Rz(q(4))*T(0,-Lu,0) ... A01-L
               *Rz(q(5))*T(0,-Ll,0) ... A12-L
               *Rz(q(6))*T(0,-Sa,0) ... A23-L
            )* T(0,0, H/2) * TNE;
    elseif params.mode == 1 % RIGHT FIXED
        TBE = TBRb * inv( ...
                Rz(q(4))*T(0,-Lu,0) ... A01-L
               *Rz(q(5))*T(0,-Ll,0) ... A12-L
               *Rz(q(6))*T(0,-Sa,0) ... A23-L
            )* T(0,0, H) ...
            *Rz(q(3))*T(0,-Lu,0) ... A01-L
            *Rz(q(2))*T(0,-Ll,0) ... A12-L
            *Rz(q(1))*T(0,-Sa,0) ... A23-L
            * TNE;

        TBF = TB0r ...
            *Rz(q(4))*T(0,-Lu,0) ... A01-L
            *Rz(q(5))*T(0,-Ll,0) ... A12-L
            *Rz(q(6))*T(0,-Sa,0) ... A23-L
            *TNE; % FIXED ANKLE
    end

    %% Homogeneous Transforms
    if params.mode == -1
        HTs.AG0_L = TB0l;
        HTs.AG1_L = HTs.AG0_L *Rz(q(3))*T(0,-Lu,0);
        HTs.AG2_L = HTs.AG1_L *Rz(q(2))*T(0,-Ll,0);
        HTs.AG3_L = HTs.AG2_L *Rz(q(1))*T(0,-Sa,0);
        HTs.AGE_L = HTs.AG3_L * TNE;

        HTs.AG0_R = HTs.AG0_L * T(0,0,-H);
        HTs.AG1_R = HTs.AG0_R *Rz(q(4))*T(0,-Lu,0);
        HTs.AG2_R = HTs.AG1_R *Rz(q(5))*T(0,-Ll,0);
        HTs.AG3_R = HTs.AG2_R *Rz(q(6))*T(0,-Sa,0);
        HTs.AGE_R = HTs.AG3_R * TNE;

        HTs.AGH = HTs.AG0_L * T(0,0,-H/2);

    elseif params.mode == 0
        HTs.AG0_L = TB0l;
        HTs.AG1_L = HTs.AG0_L *Rz(q(3))*T(0,-Lu,0);
        HTs.AG2_L = HTs.AG1_L *Rz(q(2))*T(0,-Ll,0);
        HTs.AG3_L = HTs.AG2_L *Rz(q(1))*T(0,-Sa,0);
        HTs.AGE_L = HTs.AG3_L * TNE;

        HTs.AG0_R = TB0r;
        HTs.AG1_R = HTs.AG0_R *Rz(q(4))*T(0,-Lu,0);
        HTs.AG2_R = HTs.AG1_R *Rz(q(5))*T(0,-Ll,0);
        HTs.AG3_R = HTs.AG2_R *Rz(q(6))*T(0,-Sa,0);
        HTs.AGE_R = HTs.AG3_R * TNE;

    elseif params.mode == 1
        HTs.AG0_R = TB0r;
        HTs.AG1_R = HTs.AG0_R *Rz(q(4))*T(0,-Lu,0);
        HTs.AG2_R = HTs.AG1_R *Rz(q(5))*T(0,-Ll,0);
        HTs.AG3_R = HTs.AG2_R *Rz(q(6))*T(0,-Sa,0);
        HTs.AGE_R = HTs.AG3_R * TNE;

        HTs.AG0_L = HTs.AG0_R * T(0,0,H);
        HTs.AG1_L = HTs.AG0_L * Rz(q(3))*T(0,-Lu,0);
        HTs.AG2_L = HTs.AG1_L * Rz(q(2))*T(0,-Ll,0);
        HTs.AG3_L = HTs.AG2_L * Rz(q(1))*T(0,-Sa,0);
        HTs.AGE_L = HTs.AG3_L * TNE;

        HTs.AGH = HTs.AG0_R * T(0,0,H/2);
    end

    %% End Effector Parameterisation
    % R₁₁:  cθ₁cθ₂      Ψ = atan2( R₃₂, R₃₃)
    % R₂₁:  sθ₁cθ₂      θ = atan2(-R₃₁, SQRT(R₃₂² + R₃₃²))
    % R₃₁: -sθ₂         ϕ = atan2( R₂₁, R₁₁)
    % R₃₂:  0
    % R₃₃:  cθ₂
    
    if params.mode ~= 0
        phi =   atan2( TBE(3,2), TBE(3,3) );  
        theta = atan2(-TBE(3,1), sqrt( TBE(3,2)^2 + TBE(3,3)^2 ) );
        psi =   atan2( TBE(2,1), TBE(1,1) );
        
        xe = [TBE(1:3,4);phi;theta;psi;TBF(1:3,4)];
        
    else
        Lphi   = 0;%atan2( TAEL(3,2), TAEL(3,3) );  
        Ltheta = 0;%atan2(-TAEL(3,1), sqrt( TAEL(3,2)^2 + TAEL(3,3)^2 ) );
        Lpsi   = 0;%atan2( TAEL(2,1), TAEL(1,1) );
        Rphi   = 0;%atan2( TAER(3,2), TAER(3,3) );  
        Rtheta = 0;%atan2(-TAER(3,1), sqrt( TAER(3,2)^2 + TAER(3,3)^2 ) );
        Rpsi   = 0;%atan2( TAER(2,1), TAER(1,1) );
        
        Lxe    = [TBEl(1:3,4);Lphi;Ltheta;Lpsi];
        Rxe    = [TBEl(1:3,4);Rphi;Rtheta;Rpsi];
        xe = Lxe - Rxe;
    end
end