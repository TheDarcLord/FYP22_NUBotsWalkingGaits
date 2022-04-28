function [qStar] = k_Inv(q0, xe, params)
% q = k⁻¹(xₑ)  [UTILITY] Inverse Kinematic Model.
%              Returns: q ~ Joint Variables for END EFFECTOR Postion xₑ
%              xe:      [X Y Z ϕ θ Ψ]ᵀ
%              qStar:   necessary joint variables
%
% SOLUTION: qˣ = ARG MIN (q): qᵀ W q + (k(q) - xeˣ)ᵀ K (k(q) - xeˣ)
    K = 1e8*eye(length(xe),length(xe));
    W = 1e5*eye(length(q0),length(q0));
    
    A       = [];
    b       = [];
    Aeq     = [];
    beq     = [];
    lb      = [];
    ub      = [];
    nonlcon = [];
    options = optimoptions('fmincon', ...
        'Display','notify',...
        'MaxFunctionEvaluations',1e4,...
        'MaxIterations',1e4);

    argmin = @(q) (q0 - q)'*W*(q0 - q) + ...
            ( k(q,params) - xe)'*K*(k(q, params) - xe);
    qStar = fmincon(argmin,q0,A,b,Aeq,beq,lb,ub,nonlcon,options);

end