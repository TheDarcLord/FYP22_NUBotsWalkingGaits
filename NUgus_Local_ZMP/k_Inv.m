function [qStar] = k_Inv(q0, xe, t_i, m, prms)
% q = k⁻¹(xₑ)  [UTILITY] Inverse Kinematic Model.
%              Returns: q ~ Joint Variables for END EFFECTOR Postion xₑ
%              xe:      [X Y Z ϕ θ Ψ]ᵀ
%              qStar:   necessary joint variables
%
% SOLUTION: qˣ = ARG MIN (q): qᵀ W q + (k(q) - xeˣ)ᵀ K (k(q) - xeˣ)
    Kxe  = 1e5*eye(length(xe),length(xe));
    rPnd = [m.p.x(1,t_i); prms.zc; m.p.x(4,t_i)];
    Kq   = 1e1*eye(length(q0),length(q0));
    Km   = 1e3*eye(length(xe(1:3)),length(xe(1:3)));
    Kt   = 1e1;
    vTJ  = m.glbTrj(:,t_i)-m.glbTrj(:,t_i-1);
    
    NU = [];
    options = optimoptions("fmincon", ...
        "Display",'notify',...
        "MaxFunctionEvaluations",1e5,...
        "MaxIterations",1e5, ...
        "ConstraintTolerance",1e-5,...
        "EnableFeasibilityMode",true);
    argmin = @(q)                (k(q,prms) - xe)'* Kxe * ... Xe
                                 (k(q,prms) - xe) +       ...
                            (rCoM(q,prms) - rPnd)'* Km  * ... CoM
                            (rCoM(q,prms) - rPnd) +       ... 
                                         (q0 - q)'* Kq  * ... q-diff
                                         (q0 - q) +       ...
                              vTJ'*rWaist(q,prms) * Kt  * ... Traj
                              rWaist(q,prms)'*vTJ;
    qStar = fmincon(argmin,q0,NU,NU,NU,NU,prms.lb,prms.ub,@(q)nonlcon(q,prms),options);
end