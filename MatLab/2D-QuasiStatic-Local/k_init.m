function [rGBg, rBLb, rBRb, HTs] = k_init(q, params)
% k_init  [2D Model] Initialise FKM
%       INPUT:      q0, params
%       OUTPUT:     rGBg, rBLb, rBRb, HTs
%           rGBg:   Position of BODY       in GLOBAL Coordinates [X Y Z]'
%           rBLb:   Position of LEFT SOLE  in   BODY Coordinates [X Y Z]'
%           rBRb:   Position of RIGHT SOLE in   BODY Coordinates [X Y Z]'
%           HTs:    All other Homogenous Transforms

    %% LINK VARIABLES
    Ll     = params.fibula;     % Lower Leg
    Lu     = params.femur;      % Upper Leg
    Sa     = params.tarsal;     % Sole to Ankle
    H      = params.HipWidth;   % Hip Width

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
    %% TRANSFORMS
    TNE  = Rx( pi/2);

    TB0l = @(TGBg) TGBg *Rx(-pi/2)*T(0,0,H/2);  % LEFT  '0'
    TB0r = @(TGBg) TGBg *Rx(-pi/2)*T(0,0,-H/2); % RIGHT '0'

    TBEl = TB0l(T(0,0,0)) ...
         * Rz(q(3))*T(0,-Lu,0) ...  % A01-L
         * Rz(q(2))*T(0,-Ll,0) ...  % A12-L
         * Rz(q(1))*T(0,-Sa,0) ...  % A23-L
         * TNE;                     % LEFT  sole in BODY

    TBEr = TB0r(T(0,0,0)) ...
         * Rz(q(4))*T(0,-Lu,0) ...  % A01-R
         * Rz(q(5))*T(0,-Ll,0) ...  % A12-R
         * Rz(q(6))*T(0,-Sa,0) ...  % A23-R
         * TNE;                     % RIGHT sole in BODY

    rGBg = [0;0;0] - ((TBEl(1:3,4) + TBEr(1:3,4))/2);
    TGBg = T(rGBg(1),rGBg(2),rGBg(3));
    TBEl = TB0l(TGBg) ...
         * Rz(q(3))*T(0,-Lu,0) ...  % A01-L
         * Rz(q(2))*T(0,-Ll,0) ...  % A12-L
         * Rz(q(1))*T(0,-Sa,0) ...  % A23-L
         * TNE;                     % LEFT  sole in BODY
    TBEr = TB0r(TGBg) ...
         * Rz(q(4))*T(0,-Lu,0) ...  % A01-R
         * Rz(q(5))*T(0,-Ll,0) ...  % A12-R
         * Rz(q(6))*T(0,-Sa,0) ...  % A23-R
         * TNE;                     % RIGHT sole in BODY
    rBLb = TBEl(1:3,4);
    rBRb = TBEr(1:3,4);
    
    
   %% Homogeneous Transforms
    HTs.AG0_L = TB0l(TGBg);
    HTs.AG1_L = HTs.AG0_L *Rz(q(3))*T(0,-Lu,0);
    HTs.AG2_L = HTs.AG1_L *Rz(q(2))*T(0,-Ll,0);
    HTs.AG3_L = HTs.AG2_L *Rz(q(1))*T(0,-Sa,0);
    HTs.AGE_L = HTs.AG3_L * TNE;

    HTs.AG0_R = TB0r(TGBg);
    HTs.AG1_R = HTs.AG0_R *Rz(q(4))*T(0,-Lu,0);
    HTs.AG2_R = HTs.AG1_R *Rz(q(5))*T(0,-Ll,0);
    HTs.AG3_R = HTs.AG2_R *Rz(q(6))*T(0,-Sa,0);
    HTs.AGE_R = HTs.AG3_R * TNE;
end