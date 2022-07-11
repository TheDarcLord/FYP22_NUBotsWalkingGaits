function pREF = pREF(tspan, ZMP0, ZMP1, ZMPtim)
% pREF(t)   X
%           Next Increment ...
    pREF = zeros(2,length(tspan));
    for i=1:length(pREF)
        if tspan(i) < ZMPtim
            pREF(:,i) = ZMP0;
        else
            pREF(:,i) = ZMP1;
        end
    end
end