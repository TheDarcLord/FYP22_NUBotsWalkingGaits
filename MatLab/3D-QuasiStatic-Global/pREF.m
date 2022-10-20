function [pREF, sTM] = pREF(model, params)
%   pREF(t)   X
%             Next Increment ...
    Nl       = model.Nl;                % N# Future Indexes
    stepSize = params.StepSize;         % Step Size:    m
    Q        = model.glbTrj;            % Q:            (x,y,z)
    pREF     = zeros(2,length(Q)+Nl);   % [x z]ᵀ
    r        = params.HipWidth/2;       % Radius of Circle 
    STEP     = params.mode;             % DEFINE MODE:  1 RIGHT Step 
                                        %              -1 LEFT  Step
    gradFUNC = @(A,B) (B(2) - A(2)) ...
                     /(B(1) - A(1));    % Gradient -> `m`

    %% Initialise
    accuDist = 0;           % Accumulated Distance
    A        = Q(:,1);      % A = 
    sTM      = [1; STEP];   % Step( Time & Mode )

    for i = 2:length(Q)
        accuDist = accuDist + norm(Q(:,i-1) - Q(:,i));
        pREF(:,i) = pREF(:,i-1);
        if accuDist > stepSize || i == length(Q)
            B = Q(:,i);
            M = gradFUNC(A([1 3]),B([1 3]));
            C = B([1 3]);
            % Right (+) & Left (-)
            pREF(:,i)   = C + (STEP*[M*r*sqrt(1/(1+M^2)); ...
                                      -r*sqrt(1/(1+M^2))]);
            accuDist    = 0;
            STEP = STEP * -1;
            A = B;
            sTM = [sTM [i; STEP]];
        end
    end

    for i = length(Q):length(pREF)
        pREF(:,i) = pREF(:,i-1);
    end
end