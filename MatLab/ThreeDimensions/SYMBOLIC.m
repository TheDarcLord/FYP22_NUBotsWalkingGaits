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
%% T1_12
T1_12 = Rz( q1)*    RyPI_2  *T(0,  S,  0)  ... A12
       *Rz( q2)*    RyPI_N2 *T(0, Ll,  0)  ... A23
       *Rz( q3)             *T(0, Lu,  0)  ... A34
       *Rz( q4)*    RyPI_2  *T(0,  S,  0)  ... A45
       *Rz( q5)*    RxPI_2  *T(0,  0, -S)  ... A56
       *Rz( q6)             *T(H,  0,  0)  ... A67
       *Rz( q7)*    RxPI_N2 *T(0, -S,  0)  ... A78
       *Rz( q8)*    RyPI_N2 *T(0, -S,  0)  ... A89
       *Rz( q9)             *T(0,-Lu,  0)  ... A9_10
       *Rz(q10)*    RyPI_2  *T(0,-Ll,  0)  ... A10_11
       *Rz(q11)*    RyPI_N2 *T(0, -S,  0)  ... A11_12
       *Rz(q12);

