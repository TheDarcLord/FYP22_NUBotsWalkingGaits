function [ZMPk, CoMk, model] = LIPM2D(model,index,params)
% LIPM2D:   Discretised Linear Inverted Pendulum Model
%           - Constrained to moving in `Zc` Plane associated with CoMz
%           - 
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
        X0      = model.x(:,k);

    % LTI Model
        A       = [0,  1,  0;
                   0,  0,  1;
                   0,  0,  0];
        B       = [0;  0;  1];
        C       = [1,  0,  -(zc/g)];
    % Discretised State Equations
        Ad      = [1,  T,  (T^2)/2;
                   0,  1,        T;
                   0,  0,        1];
        Cd      = C;
        Bd      = [(T^3)/6; (T^2)/2; T];
    
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
        
        K_hat = dare(A_hat,B_hat,Q_hat,R);

    % Optimal Incremental Controller - Gains:
        y_demand    = pREF(fwdT, params);
        sigmaError  = model.y(1:k) - model.pREF(1:k);
    
        gainComp =  R + B_hat'*K_hat*B_hat;
        gainCore = (gainComp \ eye(size(gainComp))) * B_hat';
        % Integral Action on Tracking Error
            Gi           = gainCore * K_hat * I_hat;
        % State Feedback
            Gx           = gainCore * K_hat * F_hat;
        % Feedforward / Preview Action
            Ac_hat       =  A_hat - B_hat * gainCore * K_hat* A_hat;
            X_hat        =  zeros(p+n,NL);
            Gp           =  zeros(p,NL);
            Gp(1)        = -Gi;
            X_hat(:,1) = -Ac_hat' * K_hat*I_hat;
        for l=2:NL
            Gp(l)        = gainCore * X_hat(:,( l - 1 ));
            X_hat(:,l) = -Ac_hat' * X_hat(:,( l - 1 ));
        end

        % Alternative Feedforward / Preview Action
            Gd      = zeros(p,NL);
            f_Gd    = @(l) -gainCore * ((Ac_hat')^(l-1) * K_hat * I_hat);
        for l=1:NL
            Gd(l)        = f_Gd(l);
        end

        % Optimal Incremental Controller:
        model.gains.Gi(k) = Gi*sum(sigmaError);
        model.gains.Gx(k) = Gx*X0;
        model.gains.Gd(k) = sum( Gp .* y_demand );
        Uk = -Gi*sum(sigmaError)    ...
             -Gx*X0                 ... 
             -sum( Gp .* y_demand );

        % Control Action + Output
        model.u(:,k)   = Uk;        % Control Action/Input
        model.y(:,k)   = Cd * X0;   % Y(k)


%         figure(13) % DEBUG ?
%             clf(13)
%             subplot(2,2,1) % VISUALISE Gains
%             hold on
%             title("Gain Magnitude @Time = " + model.t(k) + " ...")
%             plot(fwdT,abs(Gd),'k-','LineWidth',2);
%             plot(fwdT,abs(Gp),'r--','LineWidth',2);
%             plot(model.t(k),abs(Gi),'bx','MarkerSize',5,'LineWidth',2);
%             plot(model.t(k),abs(Gx(1)),'co','MarkerSize',5,'LineWidth',2);
%             legend('| Gd | - Demand',               ...
%                    '| Gp | - Demand',               ...
%                    '| Gi | - Integral',             ...
%                    '| Gx |_x - Incremental State');
%             subplot(2,2,2)  % VISUALISE Gain * Demand 
%                 hold on
%                 title("Desired Demand @Time = " + model.t(k) + " to NL");
%                 plot(fwdT,y_demand,'k-','LineWidth',2);
%                 plot(fwdT,Gd .* y_demand,'m-','LineWidth',2);
%                 plot(fwdT,Gp .* y_demand,'c-','LineWidth',1);
%                 legend('Demand( k+1 : NL )', ...
%                        'Gd ⋅ Demand( k+1 : NL )',...
%                        'Gp ⋅ Demand( k+1 : NL )');
%             subplot(2,2,3)  % VISUALISE Static Gains
%                 hold on
%                 title("Static Gains @Time = " + model.t(k));
%                 plot(model.t(1:k),model.gains.Gi(1:k),'b','LineWidth',2,'MarkerSize',10);
%                 plot(model.t(1:k),model.gains.Gx(1:k),'c','LineWidth',2,'MarkerSize',10);
%                 plot(model.t(1:k),model.gains.Gd(1:k),'m','LineWidth',2,'MarkerSize',10);
%                 legend('Gi ⋅ \Sigma Error', ...
%                        'Gx ⋅ X(k)',         ...
%                        '\SigmaGp(l)⋅y_{d}(k+l)');
%             subplot(2,2,4)  % VISUALISE Static Gains
%                 hold on
%                 title("Control Signal @ " + model.t(k));
%                 plot(model.t(1:k),model.u(1:k),'k-','LineWidth',2);
%                 legend('U(k)');
%         pause(0.01)


%    % Dynamic System
%         dx = @(t,x) A*x + B*Uk;
%         tspan  = [model.t(k) model.t(k)+T];
%         [~, X] = ode45(dx,tspan,X0);
    % Discrete System
        Xd = Ad * X0 + Bd * Uk; % X(k+1)

        if  model.t(k) < model.t(end)
            model.x(:,k+1) = Xd;            % X discrete
            %model.x(:,k+1) = X(end,:)';     % X dynamic
        end

    % Outputs for FKM & FKM⁻¹
        ZMPk = model.y(1,k);
        CoMk = model.x(1,k);
end