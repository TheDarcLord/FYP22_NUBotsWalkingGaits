function pass = plotSteps(model)
    Q = model.glbTrj;
    plot3(Q(3,:), ...                % Z
          Q(1,:), ...                % X
          zeros(1,length(Q)),'k:','LineWidth',1.5);   % Y
%     % Z X Y 
%     plot3(model.p.pREF(2,1:t_n), ...                   % Z
%           model.p.pREF(1,1:t_n), ...                   % X
%           zeros(1,length(model.p.pREF(1:t_n))),'rx');  % Y
%     % Z X Y 
%     plot3(model.p.pREF(2,1:t_n), ...                   % Z
%           model.p.pREF(1,1:t_n), ...                   % X
%           zeros(1,length(model.p.pREF(1:t_n))),'r:');  % Y

    drawnow
    pass = 1;
end