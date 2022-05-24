function pREF = pREF(t)
% pREF(t)   X
%           Next Increment ...
    stepSize = 0.075;
    pREF = zeros(1,length(t));
    for i=1:length(pREF)
        if     t(i) < 2
            pREF(i) = stepSize*0;
        elseif t(i) < 3
            pREF(i) = stepSize*1;
        elseif t(i) < 4
            pREF(i) = stepSize*2;
        elseif t(i) < 5
            pREF(i) = stepSize*3;
        elseif t(i) < 6
            pREF(i) = stepSize*4;
        elseif t(i) < 7
            pREF(i) = stepSize*5;
        else
            pREF(i) = stepSize*6;
        end
    end
end