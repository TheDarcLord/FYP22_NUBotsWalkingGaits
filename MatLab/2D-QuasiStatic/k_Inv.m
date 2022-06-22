function [qStar] = k_Inv(q0, xe, index, model, params)
% q = k⁻¹(xₑ)  [UTILITY] Inverse Kinematic Model.
%              Returns: q ~ Joint Variables for END EFFECTOR Postion xₑ
%              xe:      [X Y Z ϕ θ Ψ]ᵀ
%              qStar:   necessary joint variables
%
% SOLUTION: qˣ = ARG MIN (q): qᵀ W q + (k(q) - xeˣ)ᵀ K (k(q) - xeˣ)
    Kxe = 1e9*eye(length(xe),length(xe));
    Kq  = 1*eye(length(q0),length(q0));
    Km  = 1e9*eye(length(xe(1:3)),length(xe(1:3)));
    kN  = index - 1;

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

    if params.mode ~= 0
        R0A = zeros(3,1);

        if params.mode > 0
            R0A = model.rBRb(:,kN);
        elseif params.mode < 0
            R0A = model.rBLb(:,kN);
        end

        R0A(2) = model.rCoMb(2,kN);

        argmin = @(q) (q0 - q)'*Kq *(q0 - q) + ...
            (k_NEW(q,kN,model,params) - xe)'*Kxe*(k_NEW(q,kN,model,params) - xe) + ...
        (rCoM(q,kN,model,params) - R0A)'*Km *(rCoM(q,kN,model,params) - R0A);
    
        qStar = fmincon(argmin,q0,A,b,Aeq,beq,lb,ub,nonlcon,options);
    else
        argmin = @(q) (q0 - q)'*Kq *(q0 - q) + ...
                 (k_NEW(q,kN,model,params))'*Kxe*(k_NEW(q,kN,model,params)) + ...
    (rCoM(q,kN,model,params) - xe(1:3))'*Km *(rCoM(q,kN,model,params) - xe(1:3));
    
        qStar = fmincon(argmin,q0,A,b,Aeq,beq,lb,ub,nonlcon,options);
    end
end