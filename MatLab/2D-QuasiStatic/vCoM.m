function [c, ceq] = vCoM(q, kN, m, p)
%VCOM Summary of this function goes here
%   Detailed explanation goes here
    dt  = p.timestep;
    c   = abs((rCoM(q,kN,m,p) - m.rCoMb(:,kN))./dt) - [0.1; 0.1; 0.01];
    ceq = [];
end

