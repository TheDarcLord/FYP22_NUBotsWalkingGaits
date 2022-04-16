function [qStar] = k_Inv(q0, xe, params)
% q = k⁻¹(xₑ)  [UTILITY] Inverse Kinematic Model.
%              Returns: q ~ Joint Variables for END EFFECTOR Postion xₑ
%              xe:      [X Y Z ϕ θ Ψ]ᵀ
%              qStar:   necessary joint variables
%
% SOLUTION: qˣ = ARG MIN (q): qᵀ W q + (k(q) - xeˣ)ᵀ K (k(q) - xeˣ)
    K = 1e6*eye(length(xe),length(xe));
    W = 1*eye(length(q0),length(q0));
    
    argmin = @(q) q'*W*q + ( k(q,params) - xe)'*K*(k(q, params) - xe);
    qStar = fmincon(argmin,q0);

end