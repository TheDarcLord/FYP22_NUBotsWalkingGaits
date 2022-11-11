function [Q] = trajGenStep(xei,ZMP,indexspan,model,params)
%   [3D] Step Trajectory Generation
    %% TIME
        ti_i = indexspan(1);
        tf_i = indexspan(end);

     %% ANGLE
        A = model.glbTrj(:,indexspan(end));     % [x y z]
        B = model.glbTrj(:,indexspan(end)-1);   % [x y z]
        gradFUNC = @(A,B) (A(1) - B(1)) ...
                         /(A(3) - B(3));
        M = gradFUNC(A,B);
        THETA = atan2(-1/M,1);
        
    %% TRAJECTORY - CONTINUOUS TIME
         %% TRAJECTORY - CONTINUOUS TIME
        td  = model.tspan(tf_i) - model.tspan(ti_i);
        xef  = [ZMP(1); 0; ZMP(2); 0; THETA; 0];
        stpHght = params.StepHeight;
        
        Ay  = 2*stpHght / (td/2)^2;
        Vy  = Ay*(td/2);
    
        xeT = @(t)  [xei(1) + ((xef(1)-xei(1)) / td).*t;
                     Vy*t - Ay.*(t.^2)./2              ;
                     xei(3) + ((xef(3)-xei(3)) / td).*t;
                     0*t                               ;
                     xei(5) + ((xef(5)-xei(5)) / td).*t;
                     0*t];
        stepTime = model.tspan(indexspan)...
                 - model.tspan(ti_i)*ones(size(indexspan));

        Q = xeT(stepTime);
end