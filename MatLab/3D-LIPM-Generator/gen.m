function [Xf1,Xi2,d] = gen(Xd, Xi1, params)
% gen:      Generates walking pattern given:
%           - Previous/Current CoM position & velocity
%           - Desired next CoM position & velocity
    zc   = params.zc;
    g    = params.g;
    Ts   = params.Ts;
    Tdbl = params.Tdbl;
    Tc   = sqrt(zc/g);
    Xi2  = [0; % x
            0; % x'
            0; % y
            0];% y'
    a    = 1;
    b    = 1;

    %% Dynamic Walk of a Biped
    %   1984 - Miura 
    %% ↑ A Realtime Pattern Generator For Biped Walking
    %   2002 - Kajita et al.
    %    FINAL                        INIT
    %   | x⁽f⁾|   | Cᵀ       Tᶜ·Sᵀ | |x⁽i⁾|
    %   | v⁽f⁾| = | Sᵀ/Tᶜ     Cᵀ   | |v⁽i⁾|          (16)
    %   ... where:
    %       Cᵀ = cosh(Tˢ/Tᶜ)
    %       Sᵀ = sinh(Tˢ/Tᶜ)
    %       Tᶜ = √(Z꜀/g)
    Ct = cosh(Ts/Tc);
    St = sinh(Ts/Tc);
    Ag = [   Ct, Tc*St, 0,     0;
          St/Tc,    Ct, 0,     0;
              0,     0, Ct,    Tc*St;
              0,     0, St/Tc, Ct];
    Xf1 = Ag*Xi1;
    % To control walking speed:
    %   Final Foothold must be a function of inital condition of 
    %   the prior support phase ... 
    % When the desired STATE of the CoM || Pendulum is given [xᵈ,vᵈ] ...
    %   NORM(error) = a·||xᵈ - x⁽f⁾|| + b·||vᵈ - v⁽f⁾||
    %   ... where:
    %       a & b = arbitary weights
    %       _⁽f⁾  = final state
    %       _ᵈ    = desired state
    %   ... substituting (16):
    %       + calulating foothold to minimise NORM(error) ... Derivs = 0 !!
    %       = Proper Control Law
    %
    %   x⁽i⁾_NEXT = (a·Cᵀ·(xᵈ - Sᵀ·Tᶜ·vᵈ) + b·Sᵀ/Tᶜ·(vᵈ - Cᵀ·v⁽f⁾)) / Dᵀ
    Dt     = a*(Ct^2) + b*(St/Tc)^2;
    Xi2(1) = ( ...
        a*     Ct*(Xd(1) - St*Tc* Xd(2))   + ...
        b*  St/Tc*(Xd(2) -    Ct*Xf1(2)) ) / Dt;
    Xi2(2) = Xf1(2);    % Vx f(1) = Vx i(2) ... Velocity continuous
    Xi2(3) = ( ...
        a*     Ct*(Xd(3) - St*Tc* Xd(4))   + ...
        b*  St/Tc*(Xd(4) -    Ct*Xf1(4)) ) / Dt;
    Xi2(4) = Xf1(4);    % Vy f(1) = Vy i(2) ... Velocity continuous
    %   ... where:
    %       Dᵀ = a·(Cᵀ)² + b·(Sᵀ/Tᶜ)²
    %       
    %   To determine final foothold we need the distance travelled by
    %       the body during the double support phase.
    %   d = v⁽f⁾_PREV ·Tᵈᵇˡ

    d = [Xf1(2)*Tdbl;
         Xf1(4)*Tdbl];
end