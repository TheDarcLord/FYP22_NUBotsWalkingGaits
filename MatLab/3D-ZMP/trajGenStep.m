function [Q] = trajGenStep(ZMP,indexspan,index,model,params)
%   [3D Model] Step Trajectory Generation
%       
%       Returns:    [Q, V, A]
%       Q:  End Effector Position     as a function of Time [X  Y  Z ](t) 
%       V:  End Effector Velocity     as a function of Time [X' Y' Z'](t) 
%       A:  End Effector Acceleration as a function of Time [X" Y" Z"](t)
    
    rRX = model.r.r0Rg(1,index);
    rRY = model.r.r0Rg(2,index);
    rRZ = model.r.r0Rg(3,index);
    RRY = model.r.r0Rg(5,index);

    rLX = model.r.r0Lg(1,index);
    rLY = model.r.r0Lg(2,index);
    rLZ = model.r.r0Lg(3,index);
    RLY = model.r.r0Lg(5,index);

    %% SPECIAL MATRICES
    D  = diag(1:3,-1);  % Special D - Diag Matrix   Qunitic!        
    TT = model.tspan(indexspan).^((0:3).');  % [1;t;t²;t³;t⁴;t⁵] (t)     Quintic!

    %% TIME
        ti = indexspan(1);
        tf = indexspan(end);

    %% TANGENTS 
        vTraj = model.glbTrj(:,indexspan(end)) - ...
                model.glbTrj(:,indexspan(end)-1);
        vZec1  = [0 0 1]; % X Y Z
        dotAng =  acos( (vZec1 * vTraj)/norm(vTraj) ) - (pi/2);

    %% TRAJECTORY - CUBIC SPLINE
        if params.mode == 1                    
            q0 = [0 0; % qR(x)
                RRY 0; % qR(y)
                  0 0];% qR(z)
            ti = model.tspan(ti);
            tt0 = ti.^(0:3).';
            T0 = [tt0, D*tt0];
        elseif params.mode == -1
            q0 = [0 0; % qR(x)
                RLY 0; % qR(y)
                  0 0];% qR(z)
            ti = model.tspan(ti);
            tt0 = ti.^(0:3).';
            T0 = [tt0, D*tt0];
        end
        q2 = [0      0; % qR(x)
              dotAng 0; % qR(y)
              0      0];% qR(z)
        tf = model.tspan(tf);
        tt2 = tf.^(0:3).';
        T2 = [tt2, D*tt2];

        C = [q0 q2]/[T0 T2];

    %% TRAJECTORY - CONTINUOUS TIME
        td  = tf - ti;
        if params.mode == 1
            qi  = model.r.r0Rg(1:3,index);
        elseif params.mode == -1
            qi  = model.r.r0Lg(1:3,index);
        end
        
        qf  = [ZMP(1); 0; ZMP(2)];
        stepHeight = 0.1;
        
        Ay  = 2*stepHeight / (td/2)^2;
        Vy  = Ay*(td/2);
    
        qXYZ = @(t)  [qi(1) + ((qf(1)-qi(1)) / td).*t;
                      Vy*t - Ay.*(t.^2)./2           ;
                      qi(3) + ((qf(3)-qi(3)) / td).*t];
        tspan = 0:model.timestp:td;
        
        Q = [qXYZ(tspan);
                   C*TT];
end