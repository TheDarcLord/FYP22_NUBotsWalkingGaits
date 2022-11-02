function Q = trajGen_line(tspan, init)
    velocity    = 0.2;

    Q = init + [velocity*tspan;
                zeros(size(tspan));
                zeros(size(tspan))];
end