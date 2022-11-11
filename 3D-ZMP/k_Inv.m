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
    Kwh = 1e9;
    vTGg = model.glbTrj(:,index+1)-model.glbTrj(:,index);
    vXGg = [1; 0; 0];
    vZGg = [0; 0; 1];

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
    argmin = @(q)          (k(q,kN,model,params) - xe)'* Kxe * ... Xe
                           (k(q,kN,model,params) - xe) +       ...
      (rCoM(q,kN,model,params) - model.r.r0CoMg(:,kP))'* Km  * ... CoM
      (rCoM(q,kN,model,params) - model.r.r0CoMg(:,kP)) +       ... 
                                              (q0 - q)'* Kq  * ... q-diff
                                              (q0 - q) +       ...
                       vTGg'*rWaist(q,kN,model,params) * Kwh * ... Traj
                       rWaist(q,kN,model,params)'*vTGg +       ...
                         vXGg'*rHip(q,kN,model,params) * Kwh * ... X
                         rHip(q,kN,model,params)'*vXGg +       ...
                         vZGg'*rHip(q,kN,model,params) * Kwh * ... Z
                         rHip(q,kN,model,params)'*vZGg;
                             
    qStar = fmincon(argmin,q0,A,b,Aeq,beq,lb,ub,nonlcon,options);
end