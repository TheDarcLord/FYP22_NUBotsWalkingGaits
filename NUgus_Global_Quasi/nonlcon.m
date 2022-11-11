function [c, ceq] = nonlcon(q, iNEG, m, p)
    dt  = p.timestp;
    vEx = [1; 0; 0];
    vEz = [0; 0; 1];
    c   = abs((rCoM(q,iNEG,m,p) - m.r0CoMg(:,iNEG))./dt) - [0.1; 0.01; 0.1];
    ceq = [vEz'*rHip(q,iNEG,m,p);
           vEx'*rHip(q,iNEG,m,p)];
end

