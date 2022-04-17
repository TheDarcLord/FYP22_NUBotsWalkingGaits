%% FKM -> r161
    t1 = sym('t1'); % θ₁     ankle
    t2 = sym('t2'); % θ₂     knee
    t3 = sym('t3'); % θ₃     hip
    t4 = sym('t4'); % θ₄     hip
    t5 = sym('t5'); % θ₅     knee
    t6 = sym('t6'); % θ₆     ankle
    L_lower = sym('L_lower');
    L_upper = sym('L_upper');
    H = sym('H');

    RZ    = @(psi) ...
            [cos(psi) -sin(psi)  0  0;
             sin(psi)  cos(psi)  0  0;
              0        0         1  0;
              0        0         0  1];
    T     = @(x, y, z) ...
            [eye(3)     [x;y;z];
             zeros(1,3)      1];
    
    A13 = RZ(t1)*T(0,L_lower,0)*RZ(t2)*T(0,L_upper,0);

    T16 = RZ(t1)*T(0,L_lower,0)*RZ(t2)*T(0,L_upper,0)*RZ(t3)*T(0,0,-H)* ... TO HIP
          RZ(t4)*T(0,-L_upper,0)*RZ(t5)*T(0,-L_lower,0)*RZ(t6);

    %simplify(T16);
    simplify(A13)
