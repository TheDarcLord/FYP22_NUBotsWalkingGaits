%% SYMBOLIC TAA
T     = @(x, y, z) [    eye(3), [x;y;z] ;
                    zeros(1,3),    1   ];
R     = @(t) [cos(t), -sin(t), 0, 0;
              sin(t),  cos(t), 0, 0;
                   0,       0, 1, 0;
                   0,       0, 0, 1];

t6 = sym('t6');
t5 = sym('t5');
t4 = sym('t4');
t3 = sym('t3');
t2 = sym('t2');
t1 = sym('t1');
Ll = sym('Ll');
Lu = sym('Lu');
H  = sym('H');

T61 = R(-t6)*T(0, Ll,0)*R(-t5)*T(0, Lu,0)*R(-t4)*T(0,0,H)*...
      R(-t3)*T(0,-Lu,0)*R(-t2)*T(0,-Ll,0)*R(-t1);
simplify(T61);

%% Transforms ?
A62 = simplify( R(-t6)*T(0, Ll,0)*R(-t5)*T(0, Lu,0)*R(-t4)*T(0,0,H)*...
      R(-t3)*T(0,-Lu,0) );
A63 = simplify( R(-t6)*T(0, Ll,0)*R(-t5)*T(0, Lu,0)*R(-t4)*T(0,0,H) );
A64 = simplify( R(-t6)*T(0, Ll,0)*R(-t5)*T(0, Lu,0) );
A65 = simplify( R(-t6)*T(0, Ll,0) );
