function [Q, V, A] = trajectoryGeneration(model, params)
%   [2D Model] Forward Kinematic Model - FKM
%       
%       Returns:    [Q, V, A]
%       Q:  End Effector Position     as a function of Time [X  Y  Z ](t) 
%       V:  End Effector Velocity     as a function of Time [X' Y' Z'](t) 
%       A:  End Effector Acceleration as a function of Time [X" Y" Z"](t) 
    %% PARAMS
    H = params.HipWidth;

    %% SPECIAL MATRICES
    D = diag(1:5,-1);                  % Special D - Diag Matrix   Qunitic!

    if params.step == 1
        %% RIGHT FREE
        TT = model.tspan(1:61).^((0:5).');  %[1;t;t²;t³;t⁴;t⁵] (t) Quintic!
        
        q0 = [0 0 0;  %  X  Ẋ  Ẍ 
              0 0 0;  % qY vY aY
             -H 0 0]; % qZ vZ aZ
        t0 = 0;
        tt0 = t0.^(0:5).';
        T0 = [tt0, D*tt0, D^2*tt0];
        
        q1 = [0.25 0.22 0;  %  X  Ẋ  Ẍ 
              0.1  0    0;  % qY vY aY
             -H    0    0]; % qZ vZ aZ
        t1 = 3;
        tt1 = t1.^(0:5).';
        T1 = [tt1, D*tt1, D^2*tt1];
    
        q2 = [0.5  0 0;  %  X  Ẋ  Ẍ 
               0   0 0;  % qY vY aY
              -H   0 0]; % qZ vZ aZ
        t2 = 6;
        tt2 = t2.^(0:5).';
        T2 = [tt2, D*tt2, D^2*tt2];
    elseif params.step == 2
        %% LEFT FREE
        TT = model.tspan(62:121).^((0:5).');%[1;t;t²;t³;t⁴;t⁵] (t) Quintic!
        q0 = [0.0 0 0;  %  X  Ẋ  Ẍ 
              0.0 0 0;  % qY vY aY
              0.0 0 0]; % qZ vZ aZ
        t0 = 6.1;
        tt0 = t0.^(0:5).';
        T0 = [tt0, D*tt0, D^2*tt0];
        
        q1 = [0.5 0.22 0;  %  X  Ẋ  Ẍ 
              0.1 0    0;  % qY vY aY
              0.0 0    0]; % qZ vZ aZ
        t1 = 9;
        tt1 = t1.^(0:5).';
        T1 = [tt1, D*tt1, D^2*tt1];
    
        q2 = [1.0 0 0;  %  X  Ẋ  Ẍ 
              0.0 0 0;  % qY vY aY
              0.0 0 0]; % qZ vZ aZ
        t2 = 12;
        tt2 = t2.^(0:5).';
        T2 = [tt2, D*tt2, D^2*tt2];
    elseif params.step == 3
        %% RIGHT FREE
        TT = model.tspan(122:end).^((0:5).');%[1;t;t²;t³;t⁴;t⁵] (t) Quintic!
        q0 = [-0.5 0 0;  %  X  Ẋ  Ẍ 
              0.0 0 0;  % qY vY aY
               -H 0 0]; % qZ vZ aZ
        t0 = 12.1;
        tt0 = t0.^(0:5).';
        T0 = [tt0, D*tt0, D^2*tt0];
        
        q1 = [0.0 0.22 0;  %  X  Ẋ  Ẍ 
              0.1 0    0;  % qY vY aY
               -H 0    0]; % qZ vZ aZ
        t1 = 15;
        tt1 = t1.^(0:5).';
        T1 = [tt1, D*tt1, D^2*tt1];
    
        q2 = [0.5 0 0;  %  X  Ẋ  Ẍ 
              0.0 0 0;  % qY vY aY
               -H 0 0]; % qZ vZ aZ
        t2 = 18;
        tt2 = t2.^(0:5).';
        T2 = [tt2, D*tt2, D^2*tt2];
    end

    C = [q0 q1 q2]/[T0 T1 T2];

    Q = C*TT;
    V = C*D*TT;
    A = C*D^2*TT;
end