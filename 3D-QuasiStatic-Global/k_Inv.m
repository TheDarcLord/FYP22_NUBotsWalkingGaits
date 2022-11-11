function [qStar] = k_Inv(q0, xe, index, model, params)
% q = k⁻¹(xₑ)  [UTILITY] Inverse Kinematic Model.
%              Returns: q ~ Joint Variables for END EFFECTOR Postion xₑ
%              xe:      [X Y Z ϕ θ Ψ]ᵀ
%              qStar:   necessary joint variables
%
% SOLUTION: qˣ = ARG MIN (q): qᵀ W q + (k(q) - xeˣ)ᵀ K (k(q) - xeˣ)
    Kxe = 1e5*eye(length(xe),length(xe));
    Kq  = 1e1*eye(length(q0),length(q0));
    Km  = 1e3*eye(length(xe(1:3)),length(xe(1:3)));
    Kt  = 1e1;
    kN  = index - 1;
    vTJ = model.glbTrj(:,index)-model.glbTrj(:,index-1);

    rCoMi   = rCoM(q0, kN, model, params);

    if params.mode == -1
        rSP = model.r0Rg(1:3,kN);
    elseif params.mode == 1
        rSP = model.r0Lg(1:3,kN);
    end
        rSP(2) = params.zc;


    A       = [];
    b       = [];
    Aeq     = [];
    beq     = [];
    lb      = [];
    ub      = [];
    options = optimoptions("fmincon", ...
        "Display",'notify',...
        "MaxFunctionEvaluations",1e5,...
        "MaxIterations",1e5, ...
        "EnableFeasibilityMode",true,...
        "ConstraintTolerance",1e-2);
    argmin = @(q)                (k(q, kN, model, params) - xe)'* Kxe * ... Xe
                                 (k(q, kN, model, params) - xe) +       ...
                 (rCoM(q, kN, model, params) - rSP)'* Km  * ... CoM
                 (rCoM(q, kN, model, params) - rSP) +       ... 
                                           (q0 - q)'* Kq  * ... q-diff
                                           (q0 - q) +       ...
                              vTJ'*rWaist(q, kN, model, params) * Kt  * ... Traj
                                   rWaist(q, kN, model, params)'*vTJ;
    qStar = fmincon(argmin,q0,A,b,Aeq,beq,lb,ub,@(q)nonlcon(q,rCoMi,kN,model,params),options);
end