function pREF = pREF(t, params)
% pREF(t)   X
%           Next Increment ...
    stepSize = params.stepSize;
    [Q,V,A]  = trajGen(t);
    pREF     = zeros(2,length(Q)+100);
    r        = 0.1;
    RIGHT    = -1;
    gradFUNC = @(A,B) (B(2) - A(2))/(B(1) - A(1));
    accDist  = 0;
    lastREF  = zeros(2,1);

    for i = 2:length(Q)
        accDist = accDist + norm(Q(:,i-1) - Q(:,i));
        if accDist > stepSize
            M = gradFUNC(Q(1:2,i-1),Q(1:2,i));
            % Right (+) & Left (-)
            pREF(:,i)   = Q(1:2,i) + (RIGHT*[M*r*sqrt(1/(1+M^2)); ...
                                              -r*sqrt(1/(1+M^2))]);
            lastREF     = pREF(:,i);
            accDist     = 0;
            RIGHT = RIGHT * -1;
        end
        pREF(:,i) = lastREF;
    end

    for i = length(Q):length(pREF)
        pREF(:,i) = lastREF;
    end
end