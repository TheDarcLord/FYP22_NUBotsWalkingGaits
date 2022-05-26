function pREF = pREF(t, params)
% pREF(t)   X
%           Next Increment ...
    stepSize = params.stepSize;
    pREF = zeros(2,length(t));
    
    for i=1:length(pREF)
        if     t(i) < 2
            pREF(1,i) = stepSize*0;
            pREF(2,i) = stepSize*0;
        elseif t(i) < 3
            pREF(1,i) = stepSize*1;
            pREF(2,i) =  stepSize*1;
        elseif t(i) < 4
            pREF(1,i) = stepSize*2;
            pREF(2,i) = -stepSize*1;
        elseif t(i) < 5
            pREF(1,i) = stepSize*3;
            pREF(2,i) =  stepSize*1;
        elseif t(i) < 6
            pREF(1,i) = stepSize*4;
            pREF(2,i) = -stepSize*1;
        elseif t(i) < 7
            pREF(1,i) = stepSize*5;
            pREF(2,i) =  stepSize*1;
        else
            pREF(1,i) = stepSize*6;
            pREF(2,i) = -stepSize*1;
        end
    end
end