function [qStar] = k_Inv(q0, xe, i, m, p)
% q = k⁻¹(xₑ)  [UTILITY] Inverse Kinematic Model.
%              Returns: q ~ Joint Variables for END EFFECTOR Postion xₑ
%              xe:      [X Y Z ϕ θ Ψ]ᵀ
%              qStar:   necessary joint variables
%
% SOLUTION: qˣ = ARG MIN (q): qᵀ W q + (k(q) - xeˣ)ᵀ K (k(q) - xeˣ)
    Kxe = 1e3*eye(length(xe),length(xe));
    Kq  = 1*eye(length(q0),length(q0));
    Km  = 4*eye(length(xe(1:3)),length(xe(1:3)));
    kN  = i - 1;

    A       = [];
    b       = [];
    Aeq     = [];
    beq     = [];
    lb      = [];
    ub      = [];
    nonlcon = [];
    options = optimoptions('fmincon', ...
        'Display','notify',...
        'MaxFunctionEvaluations',1e5,...
        'MaxIterations',1e5);

    R0A = zeros(3,1);

    if p.mode > 0
        R0A = m.rBRb(:,kN);
    elseif p.mode < 0
        R0A = m.rBLb(:,kN);
    end

    R0A(2) = m.rCoMb(2,kN);

    argmin = @(q) (q0 - q)'*Kq *(q0 - q) + ...
        (k(q,kN,m,p) - xe)'*Kxe*(k(q,kN,m,p) - xe) + ...
    (rCoM(q,kN,m,p) - R0A)'*Km *(rCoM(q,kN,m,p) - R0A);

    qStar = fmincon(argmin,q0,A,b,Aeq,beq,lb,ub,nonlcon,options);
end