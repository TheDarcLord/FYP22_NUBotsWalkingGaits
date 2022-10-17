function [Q] = trajGenStep(xe,ZMP,indexspan,model,params)
%   [3D] Step Trajectory Generation
%       Using:
%           Cubic Spline for "End Effector" Orientation
%           Inital Value Problem for "End Effector" Position

    %% SPECIAL MATRIX
    D  = diag(1:3,-1);  % Special D - Diag Matrix   Qunitic!        

    %% TIME
        ti_i = indexspan(1);
        tm_i = indexspan(1) + floor((indexspan(end) - indexspan(1))/2);
        tf_i = indexspan(end);

        shftSpan = ti_i:tm_i-1;
        stepSpan = tm_i:tf_i;

        TT = model.tspan(stepSpan).^((0:3).');  % [1;t;t²;t³] (t) Cubic!

        Q_shift = xe * ones(1,length(shftSpan));

    %% TANGENTS 
        vTraj = model.glbTrj(:,indexspan(end)) - ...
                model.glbTrj(:,indexspan(end)-1);
        vZec1  = [0 0 1]; % X Y Z
        dotAng =  acos( (vZec1 * vTraj)/norm(vTraj) ) - (pi/2);

    %% TRAJECTORY - CUBIC SPLINE            
        q0 = [xe(4) 0; % qR(x)
              xe(5) 0; % qR(y)
              xe(6) 0];% qR(z)
        tm = model.tspan(tm_i);
        tt0 = tm.^(0:3).';
        T0 = [tt0, D*tt0];

        q2 = [0      0; % qR(x)
              dotAng 0; % qR(y)
              0      0];% qR(z)
        tf = model.tspan(tf_i);
        tt2 = tf.^(0:3).';
        T2 = [tt2, D*tt2];

        C = [q0 q2]/[T0 T2];
    %% TRAJECTORY - CONTINUOUS TIME
        td  = tf - tm;
        qi  = xe(1:3);
        qf  = [ZMP(1); 0; ZMP(2)];
        stpHght = params.StepHeight;
        
        Ay  = 2*stpHght / (td/2)^2;
        Vy  = Ay*(td/2);
    
        qXYZ = @(t)  [qi(1) + ((qf(1)-qi(1)) / td).*t;
                      Vy*t - Ay.*(t.^2)./2           ;
                      qi(3) + ((qf(3)-qi(3)) / td).*t];
        stepTime = model.tspan(stepSpan) - tm*ones(size(stepSpan));
        Q_step = [qXYZ(stepTime);
                   C*TT];

        Q = [Q_shift, Q_step];
end