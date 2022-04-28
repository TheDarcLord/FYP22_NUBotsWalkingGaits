function [Q, V, A] = trajectoryGeneration(model, span, params)
%   [2D Model] Forward Kinematic Model - FKM
%       
%       Returns:    [Q, V, A]
%       Q:  End Effector Position     as a function of Time [X  Y  Z ](t) 
%       V:  End Effector Velocity     as a function of Time [X' Y' Z'](t) 
%       A:  End Effector Acceleration as a function of Time [X" Y" Z"](t)

    
    %% PARAMS
    stepsize = 0.5;

    rRX = params.r0Rg(1);
    rRY = params.r0Rg(2);
    rRZ = params.r0Rg(3);

    rHx = params.r0Hg(1);

    rLX = params.r0Lg(1);
    rLY = params.r0Lg(2);
    rLZ = params.r0Lg(2);

    %% SPECIAL MATRICES
    D = diag(1:5,-1);                  % Special D - Diag Matrix   Qunitic!
    
    if params.step == -1
        %% RIGHT FREE
        TT = model.tspan(span).^((0:5).');  %[1;t;t²;t³;t⁴;t⁵] (t) Quintic!
        
        q0 = [rRX               0       0;  %  X  Ẋ  Ẍ 
              rRY               0       0;  % qY vY aY
              rRZ               0       0]; % qZ vZ aZ
        t0 = model.tspan(1);
        tt0 = t0.^(0:5).';
        T0 = [tt0, D*tt0, D^2*tt0];
        
        q1 = [rHx               0.1    0;  %  X  Ẋ  Ẍ 
              rRY+0.1           0       0;  % qY vY aY
              rRZ               0       0]; % qZ vZ aZ
        t1 = t0 + 3;
        tt1 = t1.^(0:5).';
        T1 = [tt1, D*tt1, D^2*tt1];
    
        q2 = [rLX+stepsize      0       0;  %  X  Ẋ  Ẍ 
              rRY               0       0;  % qY vY aY
              rRZ               0       0]; % qZ vZ aZ
        t2 = t1 + 3;
        tt2 = t2.^(0:5).';
        T2 = [tt2, D*tt2, D^2*tt2];
    elseif params.step == 1
        %% LEFT FREE
        TT = model.tspan(span).^((0:5).');%[1;t;t²;t³;t⁴;t⁵] (t) Quintic!
        q0 = [rLX               0       0;  %  X  Ẋ  Ẍ 
              rLY               0       0;  % qY vY aY
              rLZ               0       0]; % qZ vZ aZ
        t0 = model.tspan(62) + 0.1;
        tt0 = t0.^(0:5).';
        T0 = [tt0, D*tt0, D^2*tt0];
        
        q1 = [rHx               0.1     0;  %  X  Ẋ  Ẍ 
              rLY+0.1           0       0;  % qY vY aY
              rLZ               0       0]; % qZ vZ aZ
        t1 = t0 + 2.9;
        tt1 = t1.^(0:5).';
        T1 = [tt1, D*tt1, D^2*tt1];
    
        q2 = [rRX+stepsize      0       0;  %  X  Ẋ  Ẍ 
              rLY               0       0;  % qY vY aY
              rLZ               0       0]; % qZ vZ aZ
        t2 = t1 + 3;
        tt2 = t2.^(0:5).';
        T2 = [tt2, D*tt2, D^2*tt2];
    end

    C = [q0 q1 q2]/[T0 T1 T2];

    Q = C*TT;
    V = C*D*TT;
    A = C*D^2*TT;
end