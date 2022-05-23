function xKp1 = xKp1(xKp0, Uk, params)
% X(k+1)    X
%           Next Increment ... 
    T = params.timestep;

    %% DISCRETE STATE SPACE EQUATIONS
    Ad = [1,  T,  (T^2)/2;
         0,  1,        T;
         0,  1,        1];
    Bd = [(T^3)/6; (T^2)/2; T];

    xKp1 = Ad * xKp0 + Bd * Uk;
end