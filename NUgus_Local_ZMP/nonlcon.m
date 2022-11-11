function [c, ceq] = nonlcon(q, p)
%VCOM Summary of this function goes here
%   Detailed explanation goes here
    vEx = [1; 0; 0];
    vEz = [0; 0; 1];
    c   = [];
    ceq = [vEz'*rHip(q,p);
           vEx'*rHip(q,p)];
end

