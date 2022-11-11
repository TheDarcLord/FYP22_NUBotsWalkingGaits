function [qStar] = k_Inv(q0, xe, kt, m, p)
% q = k⁻¹(xₑ)  [UTILITY] Inverse Kinematic Model.
%              Returns: q ~ Joint Variables for END EFFECTOR Postion xₑ
%              xe:      [X Y Z ϕ θ Ψ]ᵀ
%              qStar:   necessary joint variables
%
% SOLUTION: qˣ = ARG MIN (q): qᵀ W q + (k(q) - xeˣ)ᵀ K (k(q) - xeˣ)

    kNEG = kt - 1;                         	 %
    rPnd = [m.p.x(1,kt); p.zc; m.p.x(4,kt)]; % 
    Kxe  = 1e5*eye(length(xe),length(xe));	 %
    Kq   = 1e0*eye(length(q0),length(q0));	 %     
    Km   = 1e4*eye(3,3);                     %
    Kwh  = 1e1;                              %
    vTGg = m.glbTrj(:,kt)-m.glbTrj(:,kNEG);  %
    NU   = [];  % NOT USED
    options = optimoptions("fmincon", ...
                           "Display",'notify',...
                           "MaxFunctionEvaluations",1e5,...
                           "MaxIterations",1e5, ...
                           "ConstraintTolerance",1e-1,...
                           "EnableFeasibilityMode",true);
                       
    argmin = @(q)       (k(q,kNEG,m,p) - xe)'* Kxe *... Xe
                        (k(q,kNEG,m,p) - xe) +      ...
                                    (q0 - q)'* Kq  *... q-diff
                                    (q0 - q) +      ...
                   (rCoM(q,kNEG,m,p) - rPnd)'* Km  *... rCoM v rPend
                   (rCoM(q,kNEG,m,p) - rPnd) +      ...
                    vTGg'*rWaist(q,kNEG,m,p) * Kwh *... Traj Vec
                    rWaist(q,kNEG,m,p)'*vTGg;
                             
    qStar = fmincon(argmin,q0,NU,NU,NU,NU,p.lb,p.ub,@(q)nonlcon(q,kNEG,m,p),options);
end