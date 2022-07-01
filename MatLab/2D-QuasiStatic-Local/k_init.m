function [xeL, xeR, HTs] = k_init(q, params)
% k(q)  [2D Model] Forward Kinematic Model - FKM
%       INPUT:      q
%       OUTPUT:     [xe, TAA, Transforms] for an array of 'q'
%       xe:         End Effector Pose [X Y Z ϕ θ Ψ]'
%       TAA:        Transform from ANKLE to END EFFECTOR
%       Transforms: All other Homogenous Transforms

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
    % BODY in BODY = [0,0,0]
    TNE  = Rx( pi/2);

    TB0l = Rx(-pi/2)*T(0,0,H/2);    % LEFT  '0'
    TBEl = TB0l ...
         * Rz(q(3))*T(0,-Lu,0) ...  % A01-L
         * Rz(q(2))*T(0,-Ll,0) ...  % A12-L
         * Rz(q(1))*T(0,-Sa,0) ...  % A23-L
         * TNE;                     % LEFT  sole in BODY

    TB0r = Rx(-pi/2)*T(0,0,-H/2);   % RIGHT '0'
    TBEr = TB0r ...
         * Rz(q(4))*T(0,-Lu,0) ...  % A01-L
         * Rz(q(5))*T(0,-Ll,0) ...  % A12-L
         * Rz(q(6))*T(0,-Sa,0) ...  % A23-L
         * TNE;                     % RIGHT sole in BODY

    xeL    = TBEl(1:3,4);
    xeR    = TBEr(1:3,4);
    
   %% Homogeneous Transforms
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
end