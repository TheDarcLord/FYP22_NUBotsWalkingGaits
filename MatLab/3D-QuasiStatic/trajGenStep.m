function [Q] = trajGenStep(xei,ZMP,indexspan,model,params)
%   [3D] Step Trajectory Generation
%       Using:
%           Cubic Spline for "End Effector" Orientation
%           Inital Value Problem for "End Effector" Position

    %% TIME
        ti_i = indexspan(1);
        tm_i = indexspan(1) + floor((indexspan(end) - indexspan(1))/2);
        tf_i = indexspan(end);

        shftSpan = ti_i:tm_i-1;
        stepSpan = tm_i:tf_i;

        Q_shift = xei * ones(1,length(shftSpan));

    %% ANGLE
        A = model.glbTrj(:,indexspan(end));     % [x y z]
        B = model.glbTrj(:,indexspan(end)-1);   % [x y z]
        gradFUNC = @(A,B) (A(1) - B(1)) ...
                         /(A(3) - B(3));
        M = gradFUNC(A,B);
        THETA = atan2(-1/M,1);
    %% TRAJECTORY - CONTINUOUS TIME
        td  = model.tspan(tf_i) - model.tspan(tm_i);
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
        stepTime = model.tspan(stepSpan) - model.tspan(tm_i)*ones(size(stepSpan));
        Q_step = xeT(stepTime);

        Q = [Q_shift, Q_step];
end