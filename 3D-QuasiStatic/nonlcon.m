function [c, ceq] = nonlcon(q, rCoMi, p)
%VCOM Summary of this function goes here
%   Detailed explanation goes here
    dt  = p.timestp;
    vEx = [1; 0; 0];
    vEz = [0; 0; 1];
    c   = norm((rCoM(q,p) - rCoMi)./dt) - 0.1;
    ceq = [vEz'*rHip(q,p);
           vEx'*rHip(q,p)];
end

