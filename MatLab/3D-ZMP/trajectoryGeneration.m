function [Q, V, A, STEP] = trajectoryGeneration(indexspan, index, model, params)
%   [2D Model] Trajectory Generation
%       
%       Returns:    [Q, V, A]
%       Q:  End Effector Position     as a function of Time [X  Y  Z ](t) 
%       V:  End Effector Velocity     as a function of Time [X' Y' Z'](t) 
%       A:  End Effector Acceleration as a function of Time [X" Y" Z"](t)

    
    %% PARAMS
    StepSize = params.StepSize;

    rRX = model.r.r0Rg(1,index);
    %rRY = model.r.r0Rg(2,index);
    rRZ = model.r.r0Rg(3,index);

    rLX = model.r.r0Lg(1,index);
    %rLY = model.r.r0Lg(2,index);
    rLZ = model.r.r0Lg(3,index);

    rCX = model.r.r0CoMg(1,index);
    rCY = model.r.r0CoMg(2,index);
    rCZ = model.r.r0CoMg(3);

    %% SPECIAL MATRICES
        D  = [0 0 0 0 0 0;
              1 0 0 0 0 0;
              0 2 0 0 0 0;
              0 0 3 0 0 0;
              0 0 0 4 0 0;
              0 0 0 0 5 0];                  % Special D - Diag Matrix   Qunitic!
        TT = model.tspan(indexspan).^((0:5).'); % [1;t;t²;t³;t⁴;t⁵] (t) Quintic!
    %% TIME
        t0_i = indexspan(1);
        t1_i = indexspan(ceil(length(indexspan)/2));
        t2_i = indexspan(end);
    %% TRAJECTORY OPTIONS
    if params.mode == -1                    
        qFIXED = [rLX; 0; rLZ];             % RIGHT FREE    
        q0 = [rRX               0.4     0.8;  %  X  Ẋ  Ẍ 
              0                 0.3     0.1;  % qY vY aY
              rRZ               0       0]; % qZ vZ aZ
        t0 = model.tspan(t0_i);
        tt0 = t0.^(0:5).';
        T0 = [tt0, D*tt0, D^2*tt0];
        %---------------------------------------------
        q1 = [(rLX+StepSize)/2  0.15    0.5;  %  X  Ẋ  Ẍ 
              0.1              -0.1    -0.3;  % qY vY aY
              rRZ               0       0]; % qZ vZ aZ
        t1 =  model.tspan(t1_i);
        tt1 = t1.^(0:5).';
        T1 = [tt1, D*tt1, D^2*tt1];
        %---------------------------------------------
        q2 = [rLX+StepSize      0       0;  %  X  Ẋ  Ẍ 
              0                 0       0;  % qY vY aY
              rRZ               0       0]; % qZ vZ aZ
        t2 = model.tspan(t2_i);
        tt2 = t2.^(0:5).';
        T2 = [tt2, D*tt2, D^2*tt2];
    elseif params.mode == 0                 
        % This is now return to standing.
        % Previously, it was shifting the CoM
        if rLX > rRX
            % Means right foot is behind
            rAX = rRX;
            rAZ = rRZ;
            qFIXED = [rLx; 0; rLz];             % BOTH FIXED
        else
            % Means left foot is behind
            rAX = rLX;
            rAZ = rLZ;
            qFIXED = [rRx; 0; rRz];             % Fixed
        end
        %---------------------------------------------
        q0 = [rAX               0       0;  %  X  Ẋ  Ẍ 
              rAY               0       0;  % qY vY aY
              rAZ               0       0]; % qZ vZ aZ
        t0 = model.tspan(t0_i);
        tt0 = t0.^(0:5).';
        T0 = [tt0, D*tt0, D^2*tt0];
        %---------------------------------------------
        q1 = [0.5*(rAX+rCX)     0.15    0;  %  X  Ẋ  Ẍ 
              rCY               0       0;  % qY vY aY
              0.5*(rAZ+rCZ)     0       0]; % qZ vZ aZ
        t1 = model.tspan(t1_i);
        tt1 = t1.^(0:5).';
        T1 = [tt1, D*tt1, D^2*tt1];
        %---------------------------------------------
        q2 = [rAX               0       0;  %  X  Ẋ  Ẍ 
              rCY               0       0;  % qY vY aY
              rAZ               0       0]; % qZ vZ aZ
        t2 = model.tspan(t2_i);
        tt2 = t2.^(0:5).';
        T2 = [tt2, D*tt2, D^2*tt2];
    elseif params.mode == 1                 
        qFIXED = [rRX; 0; rRZ];             % LEFT FREE
        q0 = [rLX               0.4     0.8;  %  X  Ẋ  Ẍ 
              0                 0.3     0.1;  % qY vY aY
              rLZ               0       0]; % qZ vZ aZ
        t0 = model.tspan(t0_i);
        tt0 = t0.^(0:5).';
        T0 = [tt0, D*tt0, D^2*tt0];
        %---------------------------------------------
        q1 = [(rRX+StepSize)/2  0.15    0.5;  %  X  Ẋ  Ẍ 
              0.1              -0.1    -0.3;  % qY vY aY
              rLZ               0       0]; % qZ vZ aZ
        t1 = model.tspan(t1_i);
        tt1 = t1.^(0:5).';
        T1 = [tt1, D*tt1, D^2*tt1];
        %---------------------------------------------
        q2 = [rRX+StepSize      0       0;  %  X  Ẋ  Ẍ 
              0                 0       0;  % qY vY aY
              rLZ               0       0]; % qZ vZ aZ
        t2 = model.tspan(t2_i);
        tt2 = t2.^(0:5).';
        T2 = [tt2, D*tt2, D^2*tt2];
    end

    C = [q0 q1 q2]/[T0 T1 T2];

    Q = C*TT;
    V = C*D*TT;
    A = C*D^2*TT;

    for qi=1:length(Q)
        if Q(2,qi) < 0
            Q(2,qi) = 0;
        end
    end

    STEP = [qFIXED([1 3],1) q2([1 3],1)];
end