function Q = trajGen_sin(tspan, init)
    wavelength  = 2;
    frequency   = 0.0625;
    amplitude   = 0.5;
    omega       = 2*pi*frequency;
    velocity    = frequency*wavelength;

    Q = init + [velocity*tspan;
                zeros(size(tspan));
                amplitude*sin(omega*tspan)];
end