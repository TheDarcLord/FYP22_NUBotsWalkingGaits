function [Q,V,A] = trajGenGlobal(t,initialPosition)
    %% SPECIAL MATRICES
        % Special D - Diag Matrix   Qunitic!
        D  = diag(1:5,-1);                      
        % [1;t;t²;t³;t⁴;t⁵] (t) Quintic!
        TT = t.^((0:5).'); 
    %% TRAJECTORY WAYPOINTS
        q0 = [initialPosition,zeros(3,2)]; 
        t0 = 0;
        tt0 = t0.^(0:5).';
        T0 = [tt0, D*tt0, D^2*tt0];
        %---------------------------------------------
        q1 = [0.75, 0, 0;  %  X  Ẋ  Ẍ 
              0,   0, 0;  % qY vY aY
              0.5, 0, 0]; % qZ vZ aZ
        t1 = 15;
        tt1 = t1.^(0:5).';
        T1 = [tt1, D*tt1, D^2*tt1];
        %---------------------------------------------
        q2 = [1.5, 0, 0;  %  X  Ẋ  Ẍ 
              0, 0, 0;  % qY vY aY
              0, 0, 0]; % qZ vZ aZ
        t2 = 30;
        tt2 = t2.^(0:5).';
        T2 = [tt2, D*tt2, D^2*tt2];
        %---------------------------------------------
        q3 = [2.25, 0, 0;  %  X  Ẋ  Ẍ 
                0, 0, 0;  % qY vY aY
             -0.5, 0, 0]; % qZ vZ aZ
        t3 = 45;
        tt3 = t3.^(0:5).';
        T3 = [tt3, D*tt3, D^2*tt3];
        %---------------------------------------------
        q4 = [3, 0, 0;  %  X  Ẋ  Ẍ 
              0, 0, 0;  % qY vY aY
              0, 0, 0]; % qZ vZ aZ
        t4 = 60;
        tt4 = t4.^(0:5).';
        T4 = [tt4, D*tt4, D^2*tt4];

    C = [q0 q1 q2 q3 q4]/[T0 T1 T2 T3 T4];

    Q = C*TT;
    V = C*D*TT;
    A = C*D^2*TT;
end