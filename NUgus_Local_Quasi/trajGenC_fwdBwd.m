function [Q,V,A] = trajGenC_fwdBwd(tstep,mp)
    %% HELPER FUNCTION
        avg = @(A,B,ti,tf) (B - A) / (tf - ti);
    %% SPECIAL MATRICES
        D  = diag(1:3,-1);      % Special D - Diag Matrix   Cubic!                     
    %% TRAJECTORY WAYPOINTS
        t0  = 0;
        d0  = mp;
        v0  = zeros(3,1);
        q0  = [d0,v0]; 
        tt0 = t0.^(0:3).';
        T0  = [tt0, D*tt0];
        %---------------------------------------------
        t1  = 30;
        d1  = [1.05; 0; 0.25];
        v1  = 0*avg(d0,d1,t0,t1);
        q1  = [d1, v1];
        tt1 = t1.^(0:3).';
        T1  = [tt1, D*tt1];
        %---------------------------------------------
            C_1 = [q0 q1] / [T0 T1];
            t_1 = t0:tstep:t1;
            Tt1 = t_1.^((0:3).');   % [1;t;t²;t³;t⁴;t⁵] (t)
        %---------------------------------------------
        t2  = 60;
        d2  = [0; 0; 0.5];
        v2  = 0*avg(d1,d2,t1,t2);
        q2  = [d2, v2];
        tt2 = t2.^(0:3).';
        T2  = [tt2, D*tt2];
        %---------------------------------------------
            C_2 = [q1 q2] / [T1 T2];
            t_2 = (t1+tstep):tstep:t2;
            Tt2 = t_2.^((0:3).');
        %---------------------------------------------
        t3  = 90;
        d3  = [1.05; 0; 0.75];
        v3  = 0*avg(d2,d3,t2,t3);
        q3  = [d3, v3];
        tt3 = t3.^(0:3).';
        T3  = [tt3, D*tt3];
        %---------------------------------------------
            C_3 = [q2 q3] / [T2 T3];
            t_3 = (t2+tstep):tstep:t3;
            Tt3 = t_3.^((0:3).');
        %---------------------------------------------
        t4  = 120;
        d4  = [0.0; 0; 1.0];
        v4  = 0*avg(d3,d4,t3,t4);
        q4  = [d4, v4];
        tt4 = t4.^(0:3).';
        T4  = [tt4, D*tt4];
        %---------------------------------------------
            C_4 = [q3 q4] / [T3 T4];
            t_4 = (t3+tstep):tstep:t4;
            Tt4 = t_4.^((0:3).');
    %% EVALUATING POSITIONS, VELOCITIES, ACCELERATIONS
    Q = [C_1*Tt1,     C_2*Tt2,     C_3*Tt3,     C_4*Tt4];
    V = [C_1*D*Tt1,   C_2*D*Tt2,   C_3*D*Tt3,   C_4*D*Tt4];
    A = [C_1*D^2*Tt1, C_2*D^2*Tt2, C_3*D^2*Tt3, C_4*D^2*Tt4];
end