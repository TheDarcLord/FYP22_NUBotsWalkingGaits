function y = y(xKp0, params)
% X(k+1)    X
%           Next Increment ... 
    %% Params
    T   = params.timestep;
    zc  = params.zc;
    g   = params.g;

    %% DISCRETE STATE SPACE EQUATIONS
    Cd  = [1, 0, -(zc/g)];

    y   = Cd * xKp0;
end