function [c, ceq] = vCoM(q, rCoMi, p)
%VCOM Summary of this function goes here
%   Detailed explanation goes here
    dt  = p.timestp;
    c   = abs(rCoM(q,p) - rCoMi)./dt - [0.1; 0.1; 0.1];
    ceq = [];
end

