function pREF = pREF(t, params)
% pREF(t)   X
%           Next Increment ...
    Nl       = params.Nl;
    stepSize = params.stepSize;
    [Q,~,~]  = trajGen(t);
    pREF     = zeros(2,length(Q)+Nl);
    r        = 0.1;
    RIGHT    = -1;
    gradFUNC = @(A,B) (B(2) - A(2))/(B(1) - A(1));
    midpFUNC = @(A,B) (A + B) / 2;
    accDist  = 0;
    lastREF  = zeros(2,1);
    A        = Q(:,1);


    for i = 2:length(Q)
        accDist = accDist + norm(Q(:,i-1) - Q(:,i));
        if accDist > stepSize
            B = Q(:,i);
            M = gradFUNC(A,B);
            C = midpFUNC(A(1:2),B(1:2));
            % Right (+) & Left (-)
            pREF(:,i)   = C + (RIGHT*[M*r*sqrt(1/(1+M^2)); ...
                                       -r*sqrt(1/(1+M^2))]);
            lastREF     = pREF(:,i);
            accDist     = 0;
            RIGHT = RIGHT * -1;
            A = B;
        end
        pREF(:,i) = lastREF;
    end

    for i = length(Q):length(pREF)
        pREF(:,i) = lastREF;
    end
end