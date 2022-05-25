function [ZMPk, CoMk, model] = LIPM3D(model,index,params)
% LIPM: Discretised Linear Inverted Pendulum
%   Detailed explanation goes here
    % Params
        T_hrzn  = params.timeHorizon;
        T       = params.timestep;
        NL      = params.Nl;
        g       = params.g;
        zc      = params.zc;
        k       = index;

    % Forward Time Horizon
        fwdT    = model.t(k)+T :T: model.t(k) + T_hrzn;
                 
    % Current State
        Xk      = model.x(:,k);

    % Discretised State Equations
        Ad      = [1,  T,  (T^2)/2,  0,  0,        0;   % x
                   0,  1,        T,  0,  0,        0;   % x'
                   0,  0,        1,  0,  0,        0;   % x"
                   0,  0,        0,  1,  T,  (T^2)/2;   % y
                   0,  0,        0,  0,  1,        T;   % y'
                   0,  0,        0,  0,  0,        1];  % y"
        Cd      = [1,  0,  -(zc/g), 0,  0,        0;
                   0,  0,        0, 1,  0,  -(zc/g)];
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
        % Feedforward / Preview Action
                y_demand     = pREF(fwdT, params);
                Gp_Yd        =  zeros(size(y_demand));
                Ac_hat       =  A_hat - B_hat * gainCore * K_hat* A_hat;
        if NL > 0
                X_hat        =  zeros(p+n,p,NL);
                Gp           =  zeros(p,p,NL);
                Gp(:,:,1)    = -Gi;
                X_hat(:,:,1) = -Ac_hat' * K_hat*I_hat;
                Gp_Yd(:,1)   = Gp(:,:,1) * y_demand(:,1);
            for l=2:NL
                Gp(:,:,l)    = gainCore * X_hat(:,:,( l - 1 ));
                X_hat(:,:,l) = Ac_hat'  * X_hat(:,:,( l - 1 ));
                Gp_Yd(:,l)   = Gp(:,:,l) * y_demand(:,l);
            end
        end

    % Optimal Incremental Controller:
        
        sigmaError = model.y(:,1:k) - model.pREF(:,1:k);
        Uk = -Gi*[sum(sigmaError(1,:));
                  sum(sigmaError(2,:))] ...
             -Gx*model.x(:,k)           ...
             -[sum(Gp_Yd(1,:));
               sum(Gp_Yd(2,:))];

    % Simulation
        model.u(:,k)   = Uk;
        model.y(:,k)   = Cd * Xk;
    if  model.t(k) < model.t(end)
        model.x(:,k+1) = Ad * Xk + Bd * Uk;
    end

    % Output -> X(K + 1)
        ZMPk = model.y(1,k);
        CoMk = model.x(1,k);
end