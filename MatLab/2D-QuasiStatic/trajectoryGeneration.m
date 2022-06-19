function [Q, V, A] = trajectoryGeneration(index, model, span, params)
%   [2D Model] Trajectory Generation
%       
%       Returns:    [Q, V, A]
%       Q:  End Effector Position     as a function of Time [X  Y  Z ](t) 
%       V:  End Effector Velocity     as a function of Time [X' Y' Z'](t) 
%       A:  End Effector Acceleration as a function of Time [X" Y" Z"](t)

    
    %% PARAMS
    StepSize = params.StepSize;

    rRX = model.r06g(1,index);
    rRY = model.r06g(2,index);
    rRZ = model.r06g(3,index);

    rHx = model.r0Hg(1,index);

    rLX = model.r01g(1,index);
    rLY = model.r01g(2,index);
    rLZ = model.r01g(3,index);

    rCX = model.rCoM(1,index);
    rCY = model.rCoM(2,index);
    rCZ = model.rCoM(3,index);

    %% SPECIAL MATRICES
    D  = diag(1:5,-1);                  % Special D - Diag Matrix   Qunitic!
    TT = model.tspan(span).^((0:5).'); % [1;t;t²;t³;t⁴;t⁵] (t) Quintic!
    
    %% TRAJECTORY OPTIONS
    if params.mode == -1                    % RIGHT FREE
        q0 = [rRX               0       0;  %  X  Ẋ  Ẍ 
              rRY               0       0;  % qY vY aY
              rRZ               0       0]; % qZ vZ aZ
        t0 = model.tspan(span(1));
        tt0 = t0.^(0:5).';
        T0 = [tt0, D*tt0, D^2*tt0];
        %---------------------------------------------
        q1 = [rHx               0.1    0;  %  X  Ẋ  Ẍ 
              rRY+0.1           0       0;  % qY vY aY
              rRZ               0       0]; % qZ vZ aZ
        t1 = t0 + 3;
        tt1 = t1.^(0:5).';
        T1 = [tt1, D*tt1, D^2*tt1];
        %---------------------------------------------
        q2 = [rLX+StepSize      0       0;  %  X  Ẋ  Ẍ 
              rRY               0       0;  % qY vY aY
              rRZ               0       0]; % qZ vZ aZ
        t2 = model.tspan(span(end));
        tt2 = t2.^(0:5).';
        T2 = [tt2, D*tt2, D^2*tt2];
    elseif params.mode == 0                 % BOTH FIXED
        if rLX > rRX
            rAX = rLX;
        else
            rAX = rRX;
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
              rCZ               0       0]; % qZ vZ aZ
        t1 = t0 + 3;
        tt1 = t1.^(0:5).';
        T1 = [tt1, D*tt1, D^2*tt1];
        %---------------------------------------------
        q2 = [rAX               0       0;  %  X  Ẋ  Ẍ 
              rCY               0       0;  % qY vY aY
              rCZ               0       0]; % qZ vZ aZ
        t2 = model.tspan(span(end));
        tt2 = t2.^(0:5).';
        T2 = [tt2, D*tt2, D^2*tt2];
    elseif params.mode == 1                 % LEFT FREE
        q0 = [rLX               0       0;  %  X  Ẋ  Ẍ 
              rLY               0       0;  % qY vY aY
              rLZ               0       0]; % qZ vZ aZ
        t0 = model.tspan(span(1));
        tt0 = t0.^(0:5).';
        T0 = [tt0, D*tt0, D^2*tt0];
        %---------------------------------------------
        q1 = [rHx               0.1     0;  %  X  Ẋ  Ẍ 
              rLY+0.1           0       0;  % qY vY aY
              rLZ               0       0]; % qZ vZ aZ
        t1 = t0 + 3;
        tt1 = t1.^(0:5).';
        T1 = [tt1, D*tt1, D^2*tt1];
        %---------------------------------------------
        q2 = [rRX+StepSize      0       0;  %  X  Ẋ  Ẍ 
              rLY               0       0;  % qY vY aY
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