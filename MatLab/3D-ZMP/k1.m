function [xe, r0EL, r0ER, r0H] = k1(q, index, model, params)
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

    Rzyx   = @(Rz,Ry,Rx) ...
        [ cos(Rz)*cos(Ry), -sin(Rz)*cos(Rx)+cos(Rz)*sin(Ry)*sin(Rx),...
                    sin(Rz)*sin(Rx)+cos(Rz)*sin(Ry)*cos(Rx);
          sin(Rz)*cos(Ry),  cos(Rz)*cos(Rx)+sin(Rz)*sin(Ry)*sin(Rx),...
                   -cos(Rz)*sin(Rx)+sin(Rz)*sin(Ry)*cos(Rx);
                 -sin(Ry),  cos(Ry)*sin(Rx)                        ,...
                    cos(Ry)*cos(Rx)];

    %% LINK VARIABLES
    Ll     = params.fibula;     % Lower Leg
    Lu     = params.femur;      % Upper Leg
    H      = params.HipWidth;
    S      = params.ServoSize;  % SERVO DIST
    
    A0EL    = [Rzyx(model.r.r0Lg(6,index), ...
                    model.r.r0Lg(5,index), ...
                    model.r.r0Lg(4,index)),... 
                    model.r.r0Lg(1:3,index);  % LEFT Ankle Position from 
              zeros(1,3),           1]; %           0rigin in Global
    A0ER    = [Rzyx(model.r.r0Rg(6,index), ...
                    model.r.r0Rg(5,index), ...
                    model.r.r0Rg(4,index)),... 
                    model.r.r0Rg(1:3,index);  % RIGHT Ankle Position from 
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

        % ___ MID-BEGIN ___ %
        A6Hx = H/2;
        A6Hy =   0;
        A6Hz =   0;
        A6HR = eye(3);
        A6H  = [A6HR, [A6Hx A6Hy A6Hz]'; ROW4];
        % ___  MID-END  ___ %

        A67x =  H*cos(q6);
        A67y =  H*sin(q6);
        A67z =  0;
        A67R = [cos(q6), -sin(q6), 0;
                sin(q6),  cos(q6), 0;
                      0,        0, 1];
        A67  = [A67R, [A67x A67y A67z]'; ROW4];

        A78x =  0;
        A78y =  0;
        A78z =  S;
        A78R = [cos(q7),  0, -sin(q7);
                sin(q7),  0,  cos(q7);
                      0, -1,       0];
        A78  = [A78R, [A78x A78y A78z]'; ROW4];

        A89x =  S*sin(q8);
        A89y = -S*cos(q8);
        A89z =  0;
        A89R = [0, -sin(q8), -cos(q8);
                0,  cos(q8), -sin(q8);
                1,        0,       0];
        A89  = [A89R, [A89x A89y A89z]'; ROW4];

        A910x =  Lu*sin(q9);
        A910y = -Lu*cos(q9);
        A910z =  0;
        A910R = [cos(q9), -sin(q9), 0;
                 sin(q9),  cos(q9), 0;
                 0,              0, 1];
        A910  = [A910R, [A910x A910y A910z]'; ROW4];

        A1011x =  Ll*sin(q10);
        A1011y = -Ll*cos(q10);
        A1011z =  0;
        A1011R = [cos(q10), -sin(q10), 0;
                  sin(q10),  cos(q10), 0;
                         0,         0, 1];
        A1011  = [A1011R, [A1011x A1011y A1011z]'; ROW4];

        A1112x =  S*sin(q11);
        A1112y = -S*cos(q11);
        A1112z =  0;
        A1112R = [0, -sin(q11), cos(q11);
                  0,  cos(q11), sin(q11);
                 -1,         0,        0];
        A1112  = [A1112R, [A1112x A1112y A1112z]'; ROW4];

        A12Ex =  0;
        A12Ey =  0;
        A12Ez =  0;
        A12ER = [0, -sin(q12), -cos(q12);
                 0,  cos(q12), -sin(q12);
                 1,         0,         0];
        A12E  = [A12ER, [A12Ex A12Ey A12Ez]'; ROW4];

        A06  = A0EL*AE1*A12*A23*A34*A45*A56;
        A0H  = A06*A6H;
        A0ER = A06*A67*A78*A89*A910*A1011*A1112*A12E;
        TAE  = A0ER;
    elseif params.mode == 0     % BOTH FIXED
        A01 = A0EL * AE1;

        A12x = -S*sin(q1);
        A12y =  S*cos(q1);
        A12z =  0;
        A12R = [0, -sin(q1), -cos(q1);
                0,  cos(q1), -sin(q1);
                1,        0,        0];
        A12  = [A12R, [A12x A12y A12z]'; ROW4];
        A02 = A01 * A12;

        A23x = -Ll*sin(q2);
        A23y =  Ll*cos(q2);
        A23z =  0;
        A23R = [cos(q2), -sin(q2), 0;
                sin(q2),  cos(q2), 0;
                      0,        0, 1];
        A23  = [A23R, [A23x A23y A23z]'; ROW4];
        A03 = A02 * A23;

        A34x = -Lu*sin(q3);
        A34y =  Lu*cos(q3);
        A34z =  0;
        A34R = [cos(q3), -sin(q3), 0;
                sin(q3),  cos(q3), 0;
                      0,        0, 1];
        A34  = [A34R, [A34x A34y A34z]'; ROW4];
        A04 = A03 * A34;

        A45x = -S*sin(q4);
        A45y =  S*cos(q4);
        A45z =  0;
        A45R = [0, -sin(q4),  cos(q4);
                0,  cos(q4),  sin(q4);
               -1,        0,        0];
        A45  = [A45R, [A45x A45y A45z]'; ROW4];
        A05 = A04 * A45;

        A56x = -S*sin(q5);
        A56y =  S*cos(q5);
        A56z =  0;
        A56R = [cos(q5), 0,  sin(q5);
                sin(q5), 0, -cos(q5);
                      0, 1,        0];
        A56  = [A56R, [A56x A56y A56z]'; ROW4];
        A06 = A05 * A56;

        % ___ MID-BEGIN ___ %
        A6Hx = H/2;
        A6Hy =   0;
        A6Hz =   0;
        A6HR = eye(3);
        A6H  = [A6HR, [A6Hx A6Hy A6Hz]'; ROW4];
        A0H =  A06 * A6H;
        % ___  MID-END  ___ %
        TAEL = A0H;

        A012 = A0ER * AE12;

        A1211x =  S*sin(q12);
        A1211y =  S*cos(q12);
        A1211z =  0;
        A1211R = [0, sin(q12), -cos(q12);
                  0, cos(q12),  sin(q12);
                  1,        0,         0];
        A1211  = [A1211R, [A1211x A1211y A1211z]'; ROW4];
        A011  = A012 * A1211;

        A1110x =  Ll*sin(q11);
        A1110y =  Ll*cos(q11);
        A1110z =  0;
        A1110R = [cos(q11), sin(q11), 0;
                 -sin(q11), cos(q11), 0;
                         0,        0, 1];
        A1110  = [A1110R, [A1110x A1110y A1110z]'; ROW4];
        A010  = A011 * A1110;

        A109x =  Lu*sin(q10);
        A109y =  Lu*cos(q10);
        A109z =  0;
        A109R = [cos(q10), sin(q10), 0;
                -sin(q10), cos(q10), 0;
                       0,       0,   1];
        A109  = [A109R, [A109x A109y A109z]'; ROW4];
        A09  = A010 * A109;

        A98x = S*sin(q9);
        A98y = S*cos(q9);
        A98z = 0;
        A98R = [0, sin(q9),  cos(q9);
                0, cos(q9), -sin(q9);
               -1,       0,       0];
        A98  = [A98R, [A98x A98y A98z]'; ROW4];
        A08  = A09 * A98;

        A87x =  S*sin(q8);
        A87y =  S*cos(q8);
        A87z =  0;
        A87R = [cos(q8), 0, -sin(q8);
               -sin(q8), 0, -cos(q8);
                      0, 1,        0];
        A87  = [A87R, [A87x A87y A87z]'; ROW4];
        A07  = A08 * A87;

        % ___ MID-BEGIN ___ %
        A7Hx = -H/2;
        A7Hy =    0;
        A7Hz =    0;
        A7HR = eye(3);
        A7H  = [A7HR, [A7Hx A7Hy A7Hz]'; ROW4];
        % ___  MID-END  ___ %
        TAER = A07 * A7H;
        
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

        % ___ MID-BEGIN ___ %
        A7Hx = -H/2;
        A7Hy =    0;
        A7Hz =    0;
        A7HR = eye(3);
        A7H  = [A7HR, [A7Hx A7Hy A7Hz]'; ROW4];
        % ___  MID-END  ___ %

        A76x = -H*cos(q7);
        A76y =  H*sin(q7);
        A76z =  0;
        A76R = [cos(q7), sin(q7), 0;
               -sin(q7), cos(q7), 0;
                      0,       0, 1];
        A76  = [A76R, [A76x A76y A76z]'; ROW4];

        A65x =  0;
        A65y =  0;
        A65z =  S;
        A65R = [cos(q6),  0, sin(q6);
               -sin(q6),  0, cos(q6);
                      0, -1,       0];
        A65  = [A65R, [A65x A65y A65z]'; ROW4];

        A54x = -S*sin(q5);
        A54y = -S*cos(q5);
        A54z =  0;
        A54R = [0,  sin(q5), -cos(q5);
                0,  cos(q5),  sin(q5);
                1,        0,        0];
        A54  = [A54R, [A54x A54y A54z]'; ROW4];

        A43x = -Lu*sin(q4);
        A43y = -Lu*cos(q4);
        A43z =  0;
        A43R = [cos(q4),  sin(q4), 0;
               -sin(q4),  cos(q4), 0;
                      0,        0, 1];
        A43  = [A43R, [A43x A43y A43z]'; ROW4];
        A32x = -Ll*sin(q3);
        A32y = -Ll*cos(q3);
        A32z =  0;
        A32R = [cos(q3), sin(q3), 0;
               -sin(q3), cos(q3), 0;
                      0,       0, 1];
        A32  = [A32R, [A32x A32y A32z]'; ROW4];

        A21x = -S*sin(q2);
        A21y = -S*cos(q2);
        A21z =  0;
        A21R = [0,  sin(q2),  cos(q2);
                0,  cos(q2), -sin(q2);
               -1,        0,        0];
        A21  = [A21R, [A21x A21y A21z]'; ROW4];

        A1Ex =  0;
        A1Ey =  0;
        A1Ez =  0;
        A1ER = [0, sin(q1), -cos(q1);
                0, cos(q1),  sin(q1);
                1,       0,        0];
        A1E  = [A1ER, [A1Ex A1Ey A1Ez]'; ROW4];

        A07  = A0ER*AE12*A1211*A1110*A109*A98*A87;
        A0H  = A07*A7H;
        A0EL = A07*A76*A65*A54*A43*A32*A21*A1E;
        TAE  = A0EL;
    end
    
    %% End Effector Parameterisation X Y Z R₂Ψ Rᵧθ Rₓϕ
    % R₁₁:  cθ₁cθ₂  Ψ = atan2( R₂₁, R₁₁)               Z
    % R₂₁:  sθ₁cθ₂  θ = atan2(-R₃₁, SQRT(R₃₂² + R₃₃²)) Y
    % R₃₁: -sθ₂     ϕ = atan2( R₃₂, R₃₃)               X   
    % R₃₂:  0
    % R₃₃:  cθ₂
    
    if params.mode ~= 0
        parameteriseXYZ = @(A) ...
            [A(1:3,4);                                  % X Y Z
             atan2( A(3,2), A(3,3));                    % ϕ R(x)
             atan2(-A(3,1), sqrt(A(3,2)^2 + A(3,3)^2)); % θ R(y) 
             atan2( A(2,1), A(1,1))];                   % Ψ R(z)
        xe   = parameteriseXYZ(TAE);
        r0EL = parameteriseXYZ(A0EL);
        r0ER = parameteriseXYZ(A0ER);
        r0H  = parameteriseXYZ(A0H);
    else
        Lphi   = atan2( TAEL(3,2), TAEL(3,3) );  
        Ltheta = atan2(-TAEL(3,1), sqrt( TAEL(3,2)^2 + TAEL(3,3)^2 ) );
        Lpsi   = atan2( TAEL(2,1), TAEL(1,1) );
        Rphi   = atan2( TAER(3,2), TAER(3,3) );  
        Rtheta = atan2(-TAER(3,1), sqrt( TAER(3,2)^2 + TAER(3,3)^2 ) );
        Rpsi   = atan2( TAER(2,1), TAER(1,1) );
        
        Lxe    = [TAEL(1:3,4);Lphi;Ltheta;Lpsi];
        Rxe    = [TAER(1:3,4);Rphi;Rtheta;Rpsi];
        xe     = Lxe - Rxe;
    end
end