function Q = trajGen_sin(tspan, init)
    wavelength  = 1.5;
    velocity    = 0.12/12;
    amplitude   = 0.5;
    frequency   = velocity/wavelength;
    omega       = 2*pi*frequency;
    

    Q = init + [velocity*tspan;
                zeros(size(tspan));
                amplitude*sin(omega*tspan)];
end