function rNEW = updateCoord(TBE,r)
%   updateCoord(HT, r) 
%       Update the BASIS by which a VECTOR is understood
%       Use to from BASE coord to END EFFECTOR coord
    REB = TBE(1:3,1:3)';
    rBE = TBE(1:3,4);
    
    rNEW = REB * (r - rBE);
end