function [Q, V, A] = trajectoryGeneration(model, span, params)
%   [2D Model] Trajectory Generation
%       
%       Returns:    [Q, V, A]
%       Q:  End Effector Position     as a function of Time [X  Y  Z ](t) 
%       V:  End Effector Velocity     as a function of Time [X' Y' Z'](t) 
%       A:  End Effector Acceleration as a function of Time [X" Y" Z"](t)

    
    %% PARAMS
    StepSize = params.StepSize;

    rRX = params.r0Rg(1);
    rRY = params.r0Rg(2);
    rRZ = params.r0Rg(3);

    rHx = params.r0Hg(1);

    rLX = params.r0Lg(1);
    rLY = params.r0Lg(2);
    rLZ = params.r0Lg(3);

    rCX = params.r0CoMg(1);
    rCY = params.r0CoMg(2);
    rCZ = params.r0CoMg(3);

    %% SPECIAL MATRICES
    D  = diag(1:5,-1);                  % Special D - Diag Matrix   Qunitic!
    TT = model.tspan(span).^((0:5).'); % [1;t;t²;t³;t⁴;t⁵] (t) Quintic!
    
    %% TRAJECTORY OPTIONS
    if params.mode == -1                    % RIGHT FREE
        q0 = [rRX               0       0;  %  X  Ẋ  Ẍ 
              0                 0       0;  % qY vY aY
              rRZ               0       0]; % qZ vZ aZ
        t0 = model.tspan(span(1));
        tt0 = t0.^(0:5).';
        T0 = [tt0, D*tt0, D^2*tt0];
        %---------------------------------------------
        q1 = [rHx               0.1    0;  %  X  Ẋ  Ẍ 
              0.1               0       0;  % qY vY aY
              rRZ               0       0]; % qZ vZ aZ
        t1 = t0 + 3;
        tt1 = t1.^(0:5).';
        T1 = [tt1, D*tt1, D^2*tt1];
        %---------------------------------------------
        q2 = [rLX+StepSize      0       0;  %  X  Ẋ  Ẍ 
              0                 0       0;  % qY vY aY
              rRZ               0       0]; % qZ vZ aZ
        t2 = model.tspan(span(end));
        tt2 = t2.^(0:5).';
        T2 = [tt2, D*tt2, D^2*tt2];
    elseif params.mode == 0                 % BOTH FIXED
        if rLX > rRX
            rAX = rLX;
            rAZ = rLZ;
        else
            rAX = rRX;
            rAZ = rRZ;
        end
        %---------------------------------------------
        q0 = [rCX               0       0;  %  X  Ẋ  Ẍ 
              rCY               0       0;  % qY vY aY
              rCZ               0       0]; % qZ vZ aZ
        t0 = model.tspan(span(1));
        tt0 = t0.^(0:5).';
        T0 = [tt0, D*tt0, D^2*tt0];
        %---------------------------------------------
        q1 = [0.5*(rAX+rCX)     0.1     0;  %  X  Ẋ  Ẍ 
              rCY               0       0;  % qY vY aY
              0.5*(rAZ+rCZ)     0       0]; % qZ vZ aZ
        t1 = t0 + 3;
        tt1 = t1.^(0:5).';
        T1 = [tt1, D*tt1, D^2*tt1];
        %---------------------------------------------
        q2 = [rAX               0       0;  %  X  Ẋ  Ẍ 
              rCY               0       0;  % qY vY aY
              rAZ               0       0]; % qZ vZ aZ
        t2 = model.tspan(span(end));
        tt2 = t2.^(0:5).';
        T2 = [tt2, D*tt2, D^2*tt2];
    elseif params.mode == 1                 % LEFT FREE
        q0 = [rLX               0       0;  %  X  Ẋ  Ẍ 
              0                 0       0;  % qY vY aY
              rLZ               0       0]; % qZ vZ aZ
        t0 = model.tspan(span(1));
        tt0 = t0.^(0:5).';
        T0 = [tt0, D*tt0, D^2*tt0];
        %---------------------------------------------
        q1 = [rHx               0.1     0;  %  X  Ẋ  Ẍ 
              0.1               0       0;  % qY vY aY
              rLZ               0       0]; % qZ vZ aZ
        t1 = t0 + 3;
        tt1 = t1.^(0:5).';
        T1 = [tt1, D*tt1, D^2*tt1];
        %---------------------------------------------
        q2 = [rRX+StepSize      0       0;  %  X  Ẋ  Ẍ 
              0                 0       0;  % qY vY aY
              rLZ               0       0]; % qZ vZ aZ
        t2 = model.tspan(span(end));
        tt2 = t2.^(0:5).';
        T2 = [tt2, D*tt2, D^2*tt2];
    end

    C = [q0 q1 q2]/[T0 T1 T2];

    Q = C*TT;
    V = C*D*TT;
    A = C*D^2*TT;
end