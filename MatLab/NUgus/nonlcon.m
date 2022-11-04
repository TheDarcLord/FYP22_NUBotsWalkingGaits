function [c, ceq] = nonlcon(q, rCoMi, p)
    dt  = p.timestp;
    vEx = [1; 0; 0];
    vEz = [0; 0; 1];
    c   = norm((rCoM(q,p) - rCoMi)./dt) - p.vMax;
    ceq = [vEz'*rHip(q,p);
           vEx'*rHip(q,p)];
end

