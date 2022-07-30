function [Q, V, A] = trajGenStep(ZMP,indexspan,index,model,params)
%   [3D Model] Step Trajectory Generation
%       
%       Returns:    [Q, V, A]
%       Q:  End Effector Position     as a function of Time [X  Y  Z ](t) 
%       V:  End Effector Velocity     as a function of Time [X' Y' Z'](t) 
%       A:  End Effector Acceleration as a function of Time [X" Y" Z"](t)
    
    rRX = model.r.r0Rg(1,index);
    rRY = model.r.r0Rg(2,index);
    rRZ = model.r.r0Rg(3,index);

    rLX = model.r.r0Lg(1,index);
    rLY = model.r.r0Lg(2,index);
    rLZ = model.r.r0Lg(3,index);

    %% SPECIAL MATRICES
    D  = diag(1:3,-1);  % Special D - Diag Matrix   Qunitic!        
    TT = model.tspan(indexspan).^((0:3).');  % [1;t;t²;t³;t⁴;t⁵] (t)     Quintic!

    %% TIME
        t0_i = indexspan(1);
        t1_i = indexspan(floor(length(indexspan)/2));
        t2_i = indexspan(end);
    %% TRAJECTORY OPTIONS
    if params.mode == 1                    
        q0 = [rRX 0;  %  X  Ẋ
              rRY 0;  % qY vY
              rRZ 0]; % qZ vZ
        t0 = model.tspan(t0_i);
        tt0 = t0.^(0:3).';
        T0 = [tt0, D*tt0];
        %---------------------------------------------
%         q1 = [(rRX+ZMP(1))/2  0; %  X  Ẋ
%               0               0;  % qY vY
%               (rRZ+ZMP(2))/2  0]; % qZ vZ
%         t1 =  model.tspan(t1_i);
%         tt1 = t1.^(0:3).';
%         T1 = [tt1, D*tt1];

    elseif params.mode == -1
        q0 = [rLX 0;  %  X  Ẋ
              rLY 0;  % qY vY
              rLZ 0]; % qZ vZ
        t0 = model.tspan(t0_i);
        tt0 = t0.^(0:3).';
        T0 = [tt0, D*tt0];
        %---------------------------------------------
%         q1 = [(rLX+ZMP(1))/2  0;  %  X  Ẋ
%               0               0;  % qY vY
%               (rLZ+ZMP(2))/2  0]; % qZ vZ
%         t1 = model.tspan(t1_i);
%         tt1 = t1.^(0:3).';
%         T1 = [tt1, D*tt1];
    end

    %---------------------------------------------
    q2 = [ZMP(1) 0;  %  X  Ẋ
          0      0;  % qY vY
          ZMP(2) 0]; % qZ vZ
    t2 = model.tspan(t2_i);
    tt2 = t2.^(0:3).';
    T2 = [tt2, D*tt2];

    C = [q0 q2]/[T0 T2];

    Q = C*TT;
    V = C*D*TT;
    A = C*D^2*TT;

end