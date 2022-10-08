function [qStar] = k_Inv(q0, xe, index, model, params)
% q = k⁻¹(xₑ)  [UTILITY] Inverse Kinematic Model.
%              Returns: q ~ Joint Variables for END EFFECTOR Postion xₑ
%              xe:      [X Y Z ϕ θ Ψ]ᵀ
%              qStar:   necessary joint variables
%
% SOLUTION: qˣ = ARG MIN (q): qᵀ W q + (k(q) - xeˣ)ᵀ K (k(q) - xeˣ)
    Kxe = 1e9*eye(length(xe),length(xe));
    Kq  = 1e6*eye(length(q0),length(q0));
    Km  = 1e7*eye(length(xe(1:3)),length(xe(1:3)));
    Kwh = 1e9;
    if index == length(model.tspan)
        vTGg = model.glbTrj(:,index)-model.glbTrj(:,index-1);
    else
        vTGg = model.glbTrj(:,index+1)-model.glbTrj(:,index);
    end
    vXGg = [1; 0; 0];
    vZGg = [0; 0; 1];
    
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
    argmin = @(q)                (k(q,params) - xe)'* Kxe * ... Xe
                                 (k(q,params) - xe) +       ...
                 (rCoM(q,params) - [0;params.zc;0])'* Km  * ... CoM
                 (rCoM(q,params) - [0;params.zc;0]) +       ... 
                                           (q0 - q)'* Kq  * ... q-diff
                                           (q0 - q) +       ...
                             vTGg'*rWaist(q,params) * Kwh * ... Traj
                                   rWaist(q,params)'*vTGg + ...
                               vXGg'*rHip(q,params) * Kwh * ... XuVec
                                     rHip(q,params)'*vXGg + ...
                               vZGg'*rHip(q,params) * Kwh * ... ZuVec
                                     rHip(q,params)'*vZGg;
                             
    qStar = fmincon(argmin,q0,A,b,Aeq,beq,lb,ub,nonlcon,options);
end