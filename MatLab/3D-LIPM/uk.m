function uk = uk(model, k, params)
% uk        X
%           Next Increment ... 
    %% Params
    T   = params.timestep;
    zc  = params.zc;
    g   = params.g;
    Nl  = params.Nl;

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
    common      = R + B_hat'*K_hat*B_hat;
    commonINV   = common \ eye(size(common));
    Gi          = commonINV * B_hat' * K_hat * I_hat;
    Gx          = commonINV * B_hat' * K_hat * F_hat;
    Ac          = A_hat - B_hat*common*B_hat'*K_hat*A_hat;
    Gp          = @(l) -commonINV*B_hat'* (Ac')^(l - 1) * K_hat*I_hat;
    
    %% Optimal Incremental Controller:
    sigInput = zeros(1,length(Nl));
    for j=1:Nl
        sigInput(j) = Gp(j) * pREF(model.t(k) + T*j);
    end
    sigError = model.y(1:k) - model.pREF(1:k);

    uk = -Gi*sum(sigError) -Gx*model.x(:,k) -sum(sigInput);
end