function [qStar] = k_Inv(q0, xe, index, model, prms)
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
    vTJ = model.glbTrj(:,index)-model.glbTrj(:,index-1);
    rCoMi   = rCoM(q0,prms);
    
    NU  = [];   % NOT USED

    options = optimoptions("fmincon", ...
        "Display",'notify',...
        "MaxFunctionEvaluations",1e5,...
        "MaxIterations",1e5, ...
        "EnableFeasibilityMode",true,...
        "ConstraintTolerance",1e-2);
    
    argmin = @(q)                (k(q,prms) - xe)'* Kxe * ... Xe
                                 (k(q,prms) - xe) +       ...
                   (rCoM(q,prms) - [0;prms.zc;0])'* Km  * ... CoM
                   (rCoM(q,prms) - [0;prms.zc;0]) +       ... 
                                         (q0 - q)'* Kq  * ... q-diff
                                         (q0 - q) +       ...
                              vTJ'*rWaist(q,prms) * Kt  * ... Traj
                                   rWaist(q,prms)'*vTJ;
    qStar = fmincon(argmin,q0,NU,NU,NU,NU,prms.lb,prms.ub,...
                    @(q)nonlcon(q,rCoMi,prms),options);
end