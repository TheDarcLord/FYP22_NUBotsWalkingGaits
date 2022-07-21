function [pREF, step] = pREF(t, params)
% pREF(t)   X
%           Next Increment ...
    stepSize = params.stepSize;
    [Q,V,A] = trajGen(t);
    pREF = zeros(2,length(Q));
    step = zeros(4,length(Q));
    r = 0.1;

    distFUNC = @(X1,X2,Y1,Y2) sqrt( (X2 - X1)^2 + (Y2 - Y1)^2 );
    gradFUNC = @(A,B) (B(2) - A(2))/ (B(1) - A(1));

    accDist = 0;
    last = zeros(2,1);
    lastStep = zeros(4,1);

    for i = 2:length(Q)
        accDist = accDist + distFUNC(Q(1,i-1),Q(1,i),Q(2,i-1),Q(2,i));
        if accDist > stepSize
            pREF(:,i)   = Q(1:2,i);                         % Hold in pREF
            M = gradFUNC(Q(1:2,i-1),Q(1:2,i));
            N = -1/M;
            Y = @(X) N*(X - Q(1,i)) + Q(2,i);
            A = 1;
            B = -2*Q(1,i);
            C = Q(1,i)^2 - r^2/(N^2 + 1);
            R = roots([A B C]);
            step(:,i)   = [R(1); Y(R(1)); [R(2); Y(R(2))]];
            last        = Q(1:2,i);
            lastStep    = step(:,i);
            accDist     = 0;
        end
        pREF(:,i) = last;
        step(:,i) = lastStep;
    end

end