function [qStar] = k_Inv(q0, xe, index, model, params)
% q = k⁻¹(xₑ)  [UTILITY] Inverse Kinematic Model.
%              Returns: q ~ Joint Variables for END EFFECTOR Postion xₑ
%              xe:      [X Y Z ϕ θ Ψ]ᵀ
%              qStar:   necessary joint variables
%
% SOLUTION: qˣ = ARG MIN (q): qᵀ W q + (k(q) - xeˣ)ᵀ K (k(q) - xeˣ)
    Kxe = 1e9*eye(length(xe),length(xe));
    Kq  = 1e3*eye(length(q0),length(q0));
    Km  = 1e9*eye(length(xe(1:3)),length(xe(1:3)));

    kN  = index - 1;
    if kN < 1
        kN = 1;
    end
    kP  = index;
    
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

    argmin = @(q) (q0 - q)'*Kq *(q0 - q) + ...
        (k(q,kN,model,params) - xe)'*Kxe*(k(q,kN,model,params) - xe) + ...
     (rCoM(q,kN,model,params) - model.r.r0CoMg(:,kP))'*Km*(rCoM(q,kN,model,params) - model.r.r0CoMg(:,kP));

    qStar = fmincon(argmin,q0,A,b,Aeq,beq,lb,ub,nonlcon,options);
end