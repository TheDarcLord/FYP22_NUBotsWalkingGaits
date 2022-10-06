function [Q] = trajGenStep(xe,ZMP,indexspan,model,params)
%   [3D] Step Trajectory Generation
%       Using:
%           Cubic Spline for "End Effector" Orientation
%           Inital Value Problem for "End Effector" Position

    %% SPECIAL MATRICES
    D  = diag(1:3,-1);  % Special D - Diag Matrix   Qunitic!        
    TT = model.tspan(indexspan).^((0:3).');  % [1;t;t²;t³] (t) Cubic!

    %% TIME
        ti = indexspan(1);
        tf = indexspan(end);

    %% TANGENTS 
        vTraj = model.glbTrj(:,indexspan(end)) - ...
                model.glbTrj(:,indexspan(end)-1);
        vZec1  = [0 0 1]; % X Y Z
        dotAng =  acos( (vZec1 * vTraj)/norm(vTraj) ) - (pi/2);

    %% TRAJECTORY - CUBIC SPLINE            
        q0 = [xe(4) 0; % qR(x)
              xe(5) 0; % qR(y)
              xe(6) 0];% qR(z)
        ti = model.tspan(ti);
        tt0 = ti.^(0:3).';
        T0 = [tt0, D*tt0];

        q2 = [0      0; % qR(x)
              dotAng 0; % qR(y)
              0      0];% qR(z)
        tf = model.tspan(tf);
        tt2 = tf.^(0:3).';
        T2 = [tt2, D*tt2];

        C = [q0 q2]/[T0 T2];
        
    %% TRAJECTORY - CONTINUOUS TIME
        td  = tf - ti;
        qi  = xe(1:3);
        qf  = [ZMP(1); 0; ZMP(2)];
        stpHght = params.StepHeight;
        
        Ay  = 2*stpHght / (td/2)^2;
        Vy  = Ay*(td/2);
    
        qXYZ = @(t)  [qi(1) + ((qf(1)-qi(1)) / td).*t;
                      Vy*t - Ay.*(t.^2)./2           ;
                      qi(3) + ((qf(3)-qi(3)) / td).*t];
        tspan =            model.tspan(indexspan) ...
            - ti*ones(size(model.tspan(indexspan)));
        Q = [qXYZ(tspan);
                   C*TT];
end