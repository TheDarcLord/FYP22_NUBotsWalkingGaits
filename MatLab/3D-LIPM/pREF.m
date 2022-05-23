function pREF = pREF(t)
% X(k+1)    X
%           Next Increment ... 
    if t < 2
        pREF = 0.0;
    elseif t < 4
        pREF = 0.5;
    elseif t < 6
        pREF = 1.0;
    elseif t < 8
        pREF = 1.5;
    else
        pREF = 2.0;
    end
end