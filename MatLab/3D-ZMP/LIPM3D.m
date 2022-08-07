function [ZMPk, CoMk, model] = LIPM3D(model,index,params)
% LIPM: Discretised Linear Inverted Pendulum
%   Detailed explanation goes here
    % Params
        T       = model.timestp;
        NL      = model.Nl;
        g       = params.g;
        zc      = params.zc;
        k       = index;

    % Forward Time Horizon INDEX
        %fwdT_k  = k+T_hrznI;
                 
    % Current State
        X0      = model.p.x(:,k);

    % Discretised State Equations
        Ad      = [1,  T,  (T^2)/2,  0,  0,        0;   % x
                   0,  1,        T,  0,  0,        0;   % x'
                   0,  0,        1,  0,  0,        0;   % x"
                   0,  0,        0,  1,  T,  (T^2)/2;   % z
                   0,  0,        0,  0,  1,        T;   % z'
                   0,  0,        0,  0,  0,        1];  % z"
        Cd      = [1,  0,  -(zc/g),  0,  0,        0;
                   0,  0,        0,  1,  0,  -(zc/g)];
        Bd      = [(T^3)/6,       0;
                   (T^2)/2,       0;
                         T,       0;
                         0, (T^3)/6;
                         0, (T^2)/2;
                         0,       T];
    
    % Performace Index Weights
        Qe = params.weights.Qe;
        Qx = params.weights.Qx;
        R  = params.weights.R;

    % Dimensions
        [n, ~] = size(Ad);
        [~, r] = size(Bd);
        [p, ~] = size(Cd);

    % Optimal Incremental Controller - Matrices:
        B_hat = [Cd * Bd;
                      Bd];
        F_hat = [Cd * Ad;
                      Ad];
        Q_hat = [        Qe, zeros(p,n);
                 zeros(n,p),        Qx];
        I_hat = [eye(p,p);
                 zeros(n,p)];
        A_hat = [I_hat, F_hat];
        
        K_hat = idare(A_hat,B_hat,Q_hat,R,[],[]);

    % Optimal Incremental Controller - Gains:
        gainComp =  R + B_hat'*K_hat*B_hat;
        gainCore = (gainComp \ eye(size(gainComp))) * B_hat';
        % Integral Action on Tracking Error
            Gi           = gainCore * K_hat * I_hat;
        % State Feedback
            Gx           = gainCore * K_hat * F_hat;
        % Feedforward / Preview Action - page 685 !
            Yd      = model.p.pREF(:,k+1:k+NL);
            Gp_Yd   =  zeros(size(Yd));
            Ac_hat  =  A_hat - B_hat * gainCore * K_hat* A_hat;
            Gd      = @(l) -gainCore*((Ac_hat')^(l-1))*K_hat*I_hat;
            for l=1:NL
                Gp_Yd(:,l)   = Gd(l) * Yd(:,l);
            end

    % Optimal Incremental Controller:
        
        sigmaError = model.p.y(:,1:k) - model.p.pREF(:,1:k);
        Uk = -Gi*[sum(sigmaError(1,:));
                  sum(sigmaError(2,:))] ...
             -Gx*model.p.x(:,k)           ...
             -[sum(Gp_Yd(1,:));
               sum(Gp_Yd(2,:))];

    % Simulation
        model.p.u(:,k)   = Uk;
        model.p.y(:,k)   = Cd * X0;
    if  model.tspan(k) < model.tspan(end)
        model.p.x(:,k+1) = Ad * X0 + Bd * Uk;
    end

    % Output -> X(K + 1)
        ZMPk = model.p.y(:,k);
        CoMk = model.p.x([1 4],k);
end