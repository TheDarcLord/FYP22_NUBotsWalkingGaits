function [Q] = trajGenStep(indexspan,model,params)
    %% PARAMS
        stpSize = params.StepSize;
        stpHght = params.StepHeight;
              H = params.HipWidth;
    %% TIME
        ti_i = indexspan(1);
        tm_i = indexspan(1) + floor((indexspan(end) - indexspan(1))/3);
        tf_i = indexspan(end);
    
        shftSpan = ti_i:tm_i-1;
        stepSpan = tm_i:tf_i;
    
        tf = model.tspan(tf_i);
        tm = model.tspan(tm_i);

    %% TRAJECTORY - CONTINUOUS TIME
        td  = tf - tm;

    if params.mode == -1
        qi = model.rBRb(:,ti_i-1);
        qf = model.rBLb(:,ti_i-1) + [stpSize; 0; 0];
        qXYZ = @(t)  [qi(1) + ((qf(1)-qi(1)) / td).*t;
                     (4.*t.*stpHght./td) .* (1 - t./td);
                      -H*ones(size(t))];
    elseif params.mode == 1
        qi = model.rBLb(:,ti_i-1);
        qf = model.rBRb(:,ti_i-1) + [stpSize; 0; 0];
        qXYZ = @(t)  [qi(1) + ((qf(1)-qi(1)) / td).*t;
                     (4.*t.*stpHght./td) .* (1 - t./td);
                      zeros(size(t))];
    end
        
    Q_shift = qi * ones(1,length(shftSpan));

    stepTime = model.tspan(stepSpan) - tm*ones(size(stepSpan));
    Q_step = qXYZ(stepTime);

    Q = [Q_shift, Q_step];

end