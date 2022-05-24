function uk = uk(model, k, params)
% uk        X
%           Next Increment ... 
    %% Params
    T       = params.timestep;
    T_hrzn  = params.timeHorizon;
    Nl      = params.Nl;
    zc      = params.zc;
    g       = params.g;

    fT  = model.t(k)+T :T: model.t(k) + T_hrzn;

    % Weights
    Qe = params.weights.Qe;
    Qx = params.weights.Qx;
    R  = params.weights.R;
    
    % Discretised STATE SPACE EQUATIONS
    Ad = [1,  T,  (T^2)/2;
          0,  1,        T;
          0,  1,        1];
    Bd = [(T^3)/6; (T^2)/2; T];
    Cd = [1, 0, -(zc/g)];
    % Dimensions
    [n, ~] = size(Ad);
    [~, r] = size(Bd);
    [p, ~] = size(Cd);

    %% Optimal Incremental Controller - Params:
    B_hat = [Cd * Bd;
                  Bd];
    F_hat = [Cd * Ad;
                  Ad];
    Q_hat = [        Qe, zeros(p,n);
             zeros(n,p),        Qx];
    I_hat = [eye(p,p);
             zeros(n,p)];
    A_hat = [I_hat, F_hat];

    K_hat = dare(A_hat,B_hat,Q_hat,R);

    
    %% Optimal Incremental Controller - Gains:
    common       = R + B_hat'*K_hat*B_hat;
    commonINV    = common \ eye(size(common));
    % Integral Action on Tracking Error
    Gi           = commonINV * B_hat' * K_hat * I_hat;
    % State Feedback
    Gx           = commonINV * B_hat' * K_hat * F_hat;
    % Feedforward / Preview Action
    Ac_hat       = A_hat - B_hat*common*B_hat'*K_hat*A_hat;
    X_hat        = zeros(p+n,p,Nl);
    Gp           = zeros(p,Nl);
    Gp(1)        = -Gi;
    X_hat(:,:,1) = -Ac_hat'*K_hat*I_hat;
    for l=2:Nl
        Gp(l)           = commonINV * B_hat' * X_hat(:,:,( l - 1 ));
        X_hat(:,:,l)    = -Ac_hat' * X_hat(:,:,( l - 1 ));
    end
    
    %% Optimal Incremental Controller:
    y_demand   = pREF(fT);
    sigmaError = model.y(1:k) - model.pREF(1:k);

    uk = -Gi*sum(sigmaError) -Gx*model.x(:,k) -sum( Gp .* y_demand );
end