function Q = trajGen_sin(tspan, init)
    wavelength  = 2;
    velocity    = 0.025;
    amplitude   = 0.25;
    frequency   = velocity/wavelength;
    omega       = 2*pi*frequency;

    Q = init + [velocity*tspan;
                zeros(size(tspan));
                amplitude*sin(omega*tspan)];
end