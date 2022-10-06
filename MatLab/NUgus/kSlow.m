function [HTs] = kSlow(q, params)
% k(q)  [2D Model] Forward Kinematic Model - FKM
%       
%       Returns:    [xe, TAA, Transforms] for an array of 'q'
%       xe:         End Effector Pose [X Y Z ϕ θ Ψ]'
%       TAA:        Transform from ANKLE to END EFFECTOR
%       Transforms: All other Homogenous Transforms
    
    %% HELPER FUNCTIONS
    Txyz    = @(x,y,z) [eye(3),[x,y,z]'; 0,0,0,1];
    Rz   = @(psi)   [ cos(psi), -sin(psi),         0, 0;
                      sin(psi),  cos(psi),         0, 0;
                             0,         0,         1, 0;
                             0,         0,         0, 1];

    RyNP2 = [0  0 -1 0;  0  1  0 0; 1  0  0 0; 0,0,0,1];
    RyP2  = [0  0  1 0;  0  1  0 0;-1  0  0 0; 0,0,0,1];
    RxP2  = [1  0  0 0;  0  0 -1 0; 0  1  0 0; 0,0,0,1];
    RxNP2 = [1  0  0 0;  0  0  1 0; 0 -1  0 0; 0,0,0,1];
    
    %% LINK VARIABLES
    H    = params.HipWidth;
    h2a  = params.heel2ankle;
    a2k  = params.ankle2knee;
    k2h  = params.knee2hip;
    h2w  = params.hip2waist;
    
    %% JOINT VARIABLES
    q1  = q(1);     % θ₁
    q2  = q(2);     % θ₂
    q3  = q(3);     % θ₃
    q4  = q(4);     % θ₄
    q5  = q(5);     % θ₅
    q6  = q(6);     % θ₆
    q7  = q(7);     % θ₇
    q8  = q(8);     % θ₈
    q9  = q(9);     % θ₉
    q10 = q(10);    % θ₁₀
    q11 = q(11);    % θ₁₁
    q12 = q(12);    % θ₁₂

    %% HOMOGENOUS TRANSFORM
    % TB_0 * [ A⁰₁(q₁)⋅A¹₂(q₂) ... Aᴶ⁻¹ⱼ  ] * T12_B

    TB0   = RyP2*Txyz(0,h2a,0);
    % INVERTIBLE !!!
    A01   = Rz( q1)*RyNP2;
    A12   = Rz( q2)*Txyz(a2k(1),a2k(2),a2k(3));
    A23   = Rz( q3)*Txyz(k2h(1),k2h(2),k2h(3));
    A34   = Rz( q4)*RyP2*Txyz(0,0,h2w(1));
    A45   = Rz( q5)*RxP2*Txyz(0,h2w(2),h2w(3));
    A56   = Rz( q6)*Txyz(H,0,0);
    A67   = Rz( q7)*RxNP2*Txyz(0,h2w(3),-h2w(2));
    A78   = Rz( q8)*RyNP2*Txyz(-h2w(1),0,0);
    A89   = Rz( q9)*Txyz(-k2h(1),-k2h(2),-k2h(3));
    A910  = Rz(q10)*Txyz(-a2k(1),-a2k(2),-a2k(3));
    A1011 = Rz(q11)*RyP2;
    A1112 = Rz(q12);
    % INVERTIBLE !!!
    T12B  = RyNP2*Txyz(0,-h2a,0);

    %% EXPORT
    if params.mode ==  1        % LEFT FIXED
        HTs.ABEL = eye(4);
        HTs.ALB0 = HTs.ABEL*TB0;
        HTs.A01  = HTs.ALB0*A01;
        HTs.A02  = HTs.A01 *A12;
        HTs.A03  = HTs.A02 *A23;
        HTs.A04  = HTs.A03 *A34;
        HTs.A05  = HTs.A04 *A45;
        HTs.A06  = HTs.A05 *A56;
         
        HTs.A07  = HTs.A06 *A67;
        HTs.A08  = HTs.A07 *A78;
        HTs.A09  = HTs.A08 *A89;
        HTs.A010 = HTs.A09 *A910;
        HTs.A011 = HTs.A010*A1011;
        HTs.ARB0 = HTs.A011*A1112;
        HTs.ABER = HTs.ARB0*T12B;
    elseif params.mode == -1     % RIGHT FIXED
        HTs.ABER = eye(4);
        HTs.ARB0 = HTs.ABER * inv(T12B);
        HTs.A011 = HTs.ARB0 * inv(A1112);
        HTs.A010 = HTs.A011 * inv(A1011);
        HTs.A09  = HTs.A010 * inv(A910);
        HTs.A08  = HTs.A09  * inv(A89);
        HTs.A07  = HTs.A08  * inv(A78);
    
        HTs.A06  = HTs.A07  * inv(A67);
        HTs.A05  = HTs.A06  * inv(A56);
        HTs.A04  = HTs.A05  * inv(A45);
        HTs.A03  = HTs.A04  * inv(A34);
        HTs.A02  = HTs.A03  * inv(A23);
        HTs.A01  = HTs.A02  * inv(A12);
        HTs.ALB0 = HTs.A01  * inv(A01);
        HTs.ABEL = HTs.ALB0 * inv(TB0);
    end
end