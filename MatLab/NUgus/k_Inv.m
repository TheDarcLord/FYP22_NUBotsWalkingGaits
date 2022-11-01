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
    vTJ = model.glbTrj(:,index)-model.glbTrj(:,index-1);
    rCoMi   = rCoM(q0,params);
    
    A       = [];
    b       = [];
    Aeq     = [];
    beq     = [];
    lb      = [-pi/2; -pi/4;     0; -pi/2; -pi/2; -pi; ...
                 -pi; -pi/2; -pi/2; -pi/2; -pi/4; -pi/2];
    ub      = [ pi/2;  pi/4;  pi/2;  pi/2;  pi/2; +pi; ...
                 +pi;  pi/2;  pi/2;     0;  pi/4;  pi/2];
    options = optimoptions("fmincon", ...
        "Display",'notify',...
        "MaxFunctionEvaluations",1e5,...
        "MaxIterations",1e5, ...
        "EnableFeasibilityMode",true,...
        "ConstraintTolerance",1e-2);
    argmin = @(q)                (k(q,params) - xe)'* Kxe * ... Xe
                                 (k(q,params) - xe) +       ...
                 (rCoM(q,params) - [0;params.zc;0])'* Km  * ... CoM
                 (rCoM(q,params) - [0;params.zc;0]) +       ... 
                                           (q0 - q)'* Kq  * ... q-diff
                                           (q0 - q) +       ...
                              vTJ'*rWaist(q,params) * Kt  * ... Traj
                                   rWaist(q,params)'*vTJ;
    qStar = fmincon(argmin,q0,A,b,Aeq,beq,lb,ub,@(q)nonlcon(q,rCoMi,params),options);
end