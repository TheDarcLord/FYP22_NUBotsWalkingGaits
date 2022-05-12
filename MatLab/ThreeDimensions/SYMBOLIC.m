clc

%% SYMBOLIC JOINT VARIABLES
    q1      = sym('q1');
    q2      = sym('q2');
    q3      = sym('q3');
    q4      = sym('q4');
    q5      = sym('q5');
    q6      = sym('q6');
    q7      = sym('q7');
    q8      = sym('q8');
    q9      = sym('q9');
    q10     = sym('q10');
    q11     = sym('q11');
    q12     = sym('q12');

%% SYMBOLIC LINK VARIABLES
    Ll     = sym('Ll');
    Lu     = sym('Lu');
    H      = sym('H');
    S      = sym('S');
%% HELPER FUNCTIONS
    T  = @(x,y,z)   [eye(3), [x;y;z];
                      0,0,0,      1];
    Rz = @(psi)     [cos(psi), -sin(psi), 0, 0;
                     sin(psi),  cos(psi), 0, 0;
                            0,         0, 1, 0;
                            0,         0, 0, 1];
    Ry = @(theta)   [cos(theta), 0, sin(theta), 0;
                              0, 1,          0, 0;
                    -sin(theta), 0, cos(theta), 0;
                              0, 0,          0, 1];
    Rx = @(phi)     [1,        0,         0, 0;
                     0, cos(phi), -sin(phi), 0;
                     0, sin(phi),  cos(phi), 0;
                     0,        0,         0, 1];

    RyPI_2  = [0  0  1  0;
               0  1  0  0;
              -1  0  0  0;
               0  0  0  1];
    RyPI_N2 = [0  0 -1  0;
               0  1  0  0;
               1  0  0  0;
               0  0  0  1];
    RxPI_2  = [1  0  0  0;
               0  0 -1  0;
               0  1  0  0;
               0  0  0  1];
    RxPI_N2 = [1  0  0  0;
               0  0  1  0;
               0 -1  0  0;
               0  0  0  1];

%% Homogenous Transforms:
% The approach here is to calculate one, then multiply it by the next
% while also storing it the HTs struct... 

    %% A12 - A21
    A12 = simplify(Rz( q1)*    RyPI_N2 *T( 0,  S,  0));
    A21 = simplify(Rz(-q2)*    RyPI_2  *T( 0, -S,  0));
    %% A23 - A32
    A23 = simplify(Rz( q2)             *T( 0, Ll,  0));
    A32 = simplify(Rz(-q3)             *T( 0,-Ll,  0));
    %% A34 - A43
    A34 = simplify(Rz( q3)             *T( 0, Lu,  0));
    A43 = simplify(Rz(-q4)             *T( 0,-Lu,  0));
    %% A45 - A54
    A45 = simplify(Rz( q4)*    RyPI_2  *T( 0,  S,  0));
    A54 = simplify(Rz(-q5)*    RyPI_N2 *T( 0, -S,  0));
    %% A56 - A65
    A56 = simplify(Rz( q5)*    RxPI_2  *T( 0,  0, -S));
    A65 = simplify(Rz(-q6)*    RxPI_N2 *T( 0, -S,  0));
    %% A67 - A76
    A67 = simplify(Rz( q6)             *T( H,  0,  0));
    A76 = simplify(Rz(-q7)             *T(-H,  0,  0));
    %% A78 - A87
    A78 = simplify(Rz( q7)*    RxPI_N2 *T( 0, -S,  0));
    A87 = simplify(Rz(-q8)*    RxPI_2  *T( 0,  0, -S));
    %% A89 - A98
    A89 = simplify(Rz( q8)*    RyPI_N2 *T( 0, -S,  0));
    A98 = simplify(Rz(-q9)*    RyPI_2  *T( 0,  S,  0));
    %% A910 - A109
    A910 = simplify(Rz( q9)            *T( 0,-Lu,  0));
    A109 = simplify(Rz(-q10)           *T( 0, Lu,  0));
    %% A1011 - A1110
    A1011 = simplify(Rz( q10)          *T( 0,-Ll,  0));
    A1110 = simplify(Rz(-q11)          *T( 0, Ll,  0));
    %% A1112 - A1211
    A1112 = simplify(Rz( q11)* RyPI_2  *T(0, -S,  0));
    A1211 = simplify(Rz(-q12)* RyPI_N2 *T(0,  S,  0));
    %% A12E
    A12E  = simplify(Rz( q12)*    RyPI_N2 * T(0, -0.05, 0));
    %% A1E
    A1E   = simplify(Rz(-q1) *   RyPI_N2);