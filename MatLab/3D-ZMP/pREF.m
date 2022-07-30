function [pREF, sTM] = pREF(t, model, params)
%   pREF(t)   X
%             Next Increment ...
    Nl       = model.Nl;                % N# Future Indexes
    stepSize = params.StepSize;         % Step Size:    m
    [Q,~,~]  = trajGenGlobal(t);        % Q:            (x,y)
    pREF     = zeros(2,length(Q)+Nl);   % [x y]ᵀ
    r        = 0.1;                     % Radius of Circle 
    STEP     = -1;                      % DEFINE MODE:  1 RIGHT Step 
                                        %              -1 LEFT  Step
    gradFUNC = @(A,B) (B(2) - A(2)) ...
                     /(B(1) - A(1));    % Gradient -> `m`
    midpFUNC = @(A,B) (A + B) / 2;      %

    % Initialise
    accuDist = 0;                       % Accumulated Distance
    lastREF  = zeros(2,1);              
    A        = Q(:,1);
    sTM      = [1; STEP];            % Step( Time & Mode )

    for i = 2:length(Q)
        accuDist = accuDist + norm(Q(:,i-1) - Q(:,i));
        if accuDist > stepSize
            B = Q(:,i);
            M = gradFUNC(A,B);
            C = midpFUNC(A(1:2),B(1:2));
            % Right (+) & Left (-)
            pREF(:,i)   = C + (STEP*[M*r*sqrt(1/(1+M^2)); ...
                                       -r*sqrt(1/(1+M^2))]);
            lastREF     = pREF(:,i);
            accuDist    = 0;
            STEP = STEP * -1;
            A = B;
            sTM = [sTM [i; STEP]];
        end
        pREF(:,i) = lastREF;
    end

    for i = length(Q):length(pREF)
        pREF(:,i) = lastREF;
    end
end