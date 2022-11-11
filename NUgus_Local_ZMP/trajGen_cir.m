function Q = trajGen_cir(tspan, init)
    omega = 0.47;
    r = 0.5;

    Q = init + [r*sin(omega*tspan);
                zeros(size(tspan));
                r*cos(omega*tspan) - r];
end