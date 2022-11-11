function [c, ceq] = nonlcon(q, rCoMi, i, m, p)
%VCOM Summary of this function goes here
%   Detailed explanation goes here
    dt  = m.timestp;
    vEx = [1; 0; 0];
    vEz = [0; 0; 1];
    c   = norm((rCoM(q, i, m, p) - rCoMi)./dt) - 0.1;
    ceq = [vEz'*rHip(q, i, m, p);
           vEx'*rHip(q, i, m, p)];
end

