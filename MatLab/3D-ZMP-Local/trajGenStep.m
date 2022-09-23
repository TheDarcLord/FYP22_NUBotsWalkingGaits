function [Q, V, A] = trajGenStep(xe,ZMP,indexspan,model)
%   [3D Model] Step Trajectory Generation
%       
%       Returns:    [Q, V, A]
%       Q:  End Effector Position     as a function of Time [X  Y  Z ](t) 
%       V:  End Effector Velocity     as a function of Time [X' Y' Z'](t) 
%       A:  End Effector Acceleration as a function of Time [X" Y" Z"](t)

    %% SPECIAL MATRICES
    D  = diag(1:3,-1);  % Special D - Diag Matrix   Qunitic!        
    TT = model.tspan(indexspan).^((0:3).');  % [1;t;t²;t³;t⁴;t⁵] (t)     Quintic!

    %% TIME
        t0_i = indexspan(1);
        %t1_i = indexspan(floor(length(indexspan)/2));
        t2_i = indexspan(end);

    %% TANGENTS 
        vTraj = model.glbTrj(:,indexspan(end)) - ...
                model.glbTrj(:,indexspan(end)-1);
        vZec1  = [0 0 1]; % X Y Z
        dotAng =  acos( (vZec1 * vTraj)/norm(vTraj) ) - (pi/2);

    %% TRAJECTORY OPTIONS
    q0 = [xe(1) 0; %  X  Ẋ
          xe(2) 0; % qY vY
          xe(3) 0; % qZ vZ
          xe(4) 0; % qR(x)
          xe(5) 0; % qR(y)
          xe(6) 0];% qR(z)
    t0 = model.tspan(t0_i);
    tt0 = t0.^(0:3).';
    T0 = [tt0, D*tt0];

    %---------------------------------------------
    q2 = [ZMP(1) 0; %  X  Ẋ
          0      0; % qY vY
          ZMP(2) 0; % qZ vZ
          0      0; % qR(x)
          dotAng 0; % qR(y)
          0      0];% qR(z)
    t2 = model.tspan(t2_i);
    tt2 = t2.^(0:3).';
    T2 = [tt2, D*tt2];

    C = [q0 q2]/[T0 T2];

    Q = C*TT;
    V = C*D*TT;
    A = C*D^2*TT;

end