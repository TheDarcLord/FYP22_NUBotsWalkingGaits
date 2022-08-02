function [v67g, r67g] = rWaist(q, index, model, params)
% k(q)  [2D Model] Forward Kinematic Model - FKM
%       
%       Returns:    [xe, TAA, Transforms] for an array of 'q'
%       xe:         End Effector Pose [X Y Z ϕ θ Ψ]'
%       TAA:        Transform from ANKLE to END EFFECTOR
%       Transforms: All other Homogenous Transforms
    
    %% HELPER VARS
    AE1  = [0  0  1  0;   %
            0  1  0  0;   %
           -1  0  0  0;   %
            0  0  0  1];  %
    AE12 = [0  0  1  0;   %
            0  1  0  0;   %
           -1  0  0  0;   %
            0  0  0  1];  %
    ROW4 = [0  0  0  1];

    %% LINK VARIABLES
    Ll     = params.fibula;     % Lower Leg
    Lu     = params.femur;      % Upper Leg
    H      = params.HipWidth;
    S      = params.ServoSize;  % SERVO DIST
    
    A0EL    = [eye(3), model.r.r0Lg(:,index);  % LEFT Ankle Position from 
              zeros(1,3),           1]; %           0rigin in Global
    A0ER    = [eye(3), model.r.r0Rg(:,index);  % RIGHT Ankle Position from 
              zeros(1,3),           1]; %           0rigin in Global
    
    %% JOINT VARIABLES
    q1  = q(1);     % θ₁
    q2  = q(2);     % θ₂
    q3  = q(3);     % θ₃
    q4  = q(4);     % θ₄
    q5  = q(5);     % θ₅
    q6  = q(6);     % θ₆
    q7  = q(7);     % θ₇
    q8  = q(8);     % θ₈
    q9  = q(9);     % θ₉
    q10 = q(10);    % θ₁₀
    q11 = q(11);    % θ₁₁
    q12 = q(12);    % θ₁₂

    %% HOMOGENOUS TRANSFORM
    if params.mode ==  1        % LEFT FIXED
        A12x = -S*sin(q1);
        A12y =  S*cos(q1);
        A12z =  0;
        A12R = [0, -sin(q1), -cos(q1);
                0,  cos(q1), -sin(q1);
                1,        0,        0];
        A12  = [A12R, [A12x A12y A12z]'; ROW4];

        A23x = -Ll*sin(q2);
        A23y =  Ll*cos(q2);
        A23z =  0;
        A23R = [cos(q2), -sin(q2), 0;
                sin(q2),  cos(q2), 0;
                      0,        0, 1];
        A23  = [A23R, [A23x A23y A23z]'; ROW4];

        A34x = -Lu*sin(q3);
        A34y =  Lu*cos(q3);
        A34z =  0;
        A34R = [cos(q3), -sin(q3), 0;
                sin(q3),  cos(q3), 0;
                      0,        0, 1];
        A34  = [A34R, [A34x A34y A34z]'; ROW4];

        A45x = -S*sin(q4);
        A45y =  S*cos(q4);
        A45z =  0;
        A45R = [0, -sin(q4),  cos(q4);
                0,  cos(q4),  sin(q4);
               -1,        0,        0];
        A45  = [A45R, [A45x A45y A45z]'; ROW4];

        A56x = -S*sin(q5);
        A56y =  S*cos(q5);
        A56z =  0;
        A56R = [cos(q5), 0,  sin(q5);
                sin(q5), 0, -cos(q5);
                      0, 1,        0];
        A56  = [A56R, [A56x A56y A56z]'; ROW4];

        A67x =  H*cos(q6);
        A67y =  H*sin(q6);
        A67z =  0;
        A67R = [cos(q6), -sin(q6), 0;
                sin(q6),  cos(q6), 0;
                      0,        0, 1];
        A67  = [A67R, [A67x A67y A67z]'; ROW4];

        A06  = A0EL*AE1*A12*A23*A34*A45*A56;
        A07  = A06*A67; 

    elseif params.mode == -1     % RIGHT FIXED
        A1211x =  S*sin(q12);
        A1211y =  S*cos(q12);
        A1211z =  0;
        A1211R = [0,  sin(q12),  -cos(q12);
                  0,  cos(q12),   sin(q12);
                  1,         0,          0];
        A1211  = [A1211R, [A1211x A1211y A1211z]'; ROW4];
        A1110x =  Ll*sin(q11);
        A1110y =  Ll*cos(q11);
        A1110z =  0;
        A1110R = [cos(q11), sin(q11), 0;
                 -sin(q11), cos(q11), 0;
                         0,        0, 1];
        A1110  = [A1110R, [A1110x A1110y A1110z]'; ROW4];
    
        A109x =  Lu*sin(q10);
        A109y =  Lu*cos(q10);
        A109z =  0;
        A109R = [cos(q10), sin(q10), 0;
                -sin(q10), cos(q10), 0;
                       0,       0,   1];
        A109  = [A109R, [A109x A109y A109z]'; ROW4];
    
        A98x = S*sin(q9);
        A98y = S*cos(q9);
        A98z = 0;
        A98R = [0, sin(q9),  cos(q9);
                0, cos(q9), -sin(q9);
               -1,       0,       0];
        A98  = [A98R, [A98x A98y A98z]'; ROW4];
    
        A87x =  S*sin(q8);
        A87y =  S*cos(q8);
        A87z =  0;
        A87R = [cos(q8), 0, -sin(q8);
               -sin(q8), 0, -cos(q8);
                      0, 1,        0];
        A87  = [A87R, [A87x A87y A87z]'; ROW4];
    
        A76x = -H*cos(q7);
        A76y =  H*sin(q7);
        A76z =  0;
        A76R = [cos(q7), sin(q7), 0;
               -sin(q7), cos(q7), 0;
                      0,       0, 1];
        A76  = [A76R, [A76x A76y A76z]'; ROW4];

        A07  = A0ER*AE12*A1211*A1110*A109*A98*A87;
        A06  = A07*A76; 
    end

    r67g = [A06(1:3,4) A07(1:3,4)];
    v67g = A07([1 3 2],4) - A06([1 3 2],4);
end