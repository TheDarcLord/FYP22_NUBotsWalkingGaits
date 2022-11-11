function [vHvert] = rHip(q, params)
% k(q)  [2D Model] Forward Kinematic Model - FKM
%       
%       Returns:    [xe, TAA, Transforms] for an array of 'q'
%       xe:         End Effector Pose [X Y Z ϕ θ Ψ]'
%       TAA:        Transform from ANKLE to END EFFECTOR
%       Transforms: All other Homogenous Transforms
    
    %% LINK VARIABLES
    Ll     = params.fibula;     % Lower Leg
    Lu     = params.femur;      % Upper Leg
    H      = params.HipWidth;
    S      = params.ServoSize;  % SERVO DIST
    
    %% HOMOGENOUS TRANSFORM
    if params.mode ==  1        % LEFT FIXED
        % JOINT VARIABLES
        c1   = cos(q(1));
        s1   = sin(q(1));
        c2   = cos(q(2));
        s2   = sin(q(2));
        c23  = cos(q(2)+q(3));
        s23  = sin(q(2)+q(3));
        c234 = cos(q(2)+q(3)+q(4));
        s234 = sin(q(2)+q(3)+q(4));
        s5   = sin(q(5));
        c5   = cos(q(5));

        TB0   = [0,0,1,0; 0,1,0,0;
                -1,0,0,0; 0,0,0,1];
        % INVERTIBLE !!!
        A04 = [c1, -s1*c234, -s1*s234, -s1*(Lu*c23 +Ll*c2 +S*(c234+1));
               s1,  c1*c234,  c1*s234,  c1*(Lu*c23 +Ll*c2 +S*(c234+1));
                0,    -s234,     c234,  -1*(Lu*s23 +Ll*s2 +S*s234);
                0,        0,        0,   1];
        A45 = [c5, 0,  s5, -S*s5;
               s5, 0, -c5,  S*c5;
               0, 1,    0,     0;
               0, 0,    0,     1];
        % INVERTIBLE !!!
        
        A  = TB0*A04*A45;
    elseif params.mode == -1     % RIGHT FIXED
        % JOINT VARIABLES
        s7   = sin(q(7));
        c7   = cos(q(7));
        s8   = sin(q(8));
        c8   = cos(q(8));
        s901 = sin(q(9)  +q(10)  +q(11));
        c901 = cos(q(9)  +q(10)  +q(11));
        s101 = sin(q(10) +q(11));
        c101 = cos(q(10) +q(11));
        s11  = sin(q(11));
        c11  = cos(q(11));
        c12  = cos(q(12));
        s12  = sin(q(12));
        % INVERTIBLE !!!
        A128 = [-s12*s901, s12*c901, -c12, (Lu*c101 +Ll*c11 +S)*s12;
                -c12*s901, c12*c901,  s12, (Lu*c101 +Ll*c11 +S)*c12;
                     c901,     s901,    0, (Lu*s101 +Ll*s11);
                        0,        0,    0, 1];
        A86 = [  -s7,     c7,   0,  0;
              -c7*s8, -s7*s8, -c8,  S*(c8 + 1);
              -c7*c8, -c8*s7,  s8, -S*s8;
                   0,      0,   0,  1];
        % INVERTIBLE !!!
        TB0   = [0,0,1,0; 0,1,0,0;
                -1,0,0,0; 0,0,0,1];
        
        A  = TB0*A128*A86;
        
    end

    Ba = A*[eye(3),[0;0;1];
                  0,0,0,1];
    vHvert = A(1:3,4) - Ba(1:3,4);
end