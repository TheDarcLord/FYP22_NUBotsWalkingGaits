function [Q, V, A] = trajectoryGeneration(model, params)
%   [2D Model] Forward Kinematic Model - FKM
%       
%       Returns:    [Q, V, A]
%       Q:  End Effector Position     as a function of Time [X  Y  Z ](t) 
%       V:  End Effector Velocity     as a function of Time [X' Y' Z'](t) 
%       A:  End Effector Acceleration as a function of Time [X" Y" Z"](t) 
    
    %% SPECIAL MATRICES
    D = diag(1:5,-1);               % Special D - Diag Matrix   Qunitic!
    TT = model.tspan.^((0:5).');    % [1 t t² t³ t⁴ t⁵]' (t)    Quintic!

    %% PARAMS
    H = params.HipWidth;
    SL = params.spanLow;
    SH = params.spanHigh;
    
    %% RIGHT FREE
    Rq0 = [0 0 0;  %  X  Ẋ  Ẍ 
          0 0 0;  % qY vY aY
         -H 0 0]; % qZ vZ aZ
    Rt0 = 0;
    Rtt0 = Rt0.^(0:5).';
    RT0 = [Rtt0, D*Rtt0, D^2*Rtt0];
    
    Rq1 = [0.25 0 0;  %  X  Ẋ  Ẍ 
           0.1  0 0;  % qY vY aY
          -H    0 0]; % qZ vZ aZ
    Rt1 = 3;
    Rtt1 = Rt1.^(0:5).';
    RT1 = [Rtt1, D*Rtt1, D^2*Rtt1];

    Rq2 = [0.5 0 0;  %  X  Ẋ  Ẍ 
           0   0 0;  % qY vY aY
          -H   0 0]; % qZ vZ aZ
    Rt2 = 6;
    Rtt2 = Rt2.^(0:5).';
    RT2 = [Rtt2, D*Rtt2, D^2*Rtt2];
    
    RC = [Rq0 Rq1 Rq2]/[RT0 RT1 RT2];

    %% LEFT FREE
    Lq0 = [-0.5 0 0;  %  X  Ẋ  Ẍ 
              0 0 0;  % qY vY aY
              0 0 0]; % qZ vZ aZ
    Lt0 = 6;
    Ltt0 = Lt0.^(0:5).';
    LT0 = [Ltt0, D*Ltt0, D^2*Ltt0];
    
    Lq1 = [0     0 0;  %  X  Ẋ  Ẍ 
           0.1   0 0;  % qY vY aY
           0     0 0]; % qZ vZ aZ
    Lt1 = 8;
    Ltt1 = Lt1.^(0:5).';
    LT1 = [Ltt1, D*Ltt1, D^2*Ltt1];

    Lq2 = [0.5 0 0;  %  X  Ẋ  Ẍ 
           0   0 0;  % qY vY aY
           0   0 0]; % qZ vZ aZ
    Lt2 = 11;
    Ltt2 = Lt2.^(0:5).';
    LT2 = [Ltt2, D*Ltt2, D^2*Ltt2];

    Lq3 = [0.5 0 0;  %  X  Ẋ  Ẍ 
           0   0 0;  % qY vY aY
           0   0 0]; % qZ vZ aZ
    Lt3 = 12;
    Ltt3 = Lt3.^(0:5).';
    LT3 = [Ltt3, D*Ltt3, D^2*Ltt3];

    LC = [Lq0 Lq1 Lq2 Lq3]/[LT0 LT1 LT2 LT3];
   
    Q = [RC*TT(:,1:SL), LC*TT(:,SH:end)];
    V = RC*D*TT;
    A = RC*D^2*TT;
    
end