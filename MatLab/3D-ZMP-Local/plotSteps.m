function pass = plotSteps(t_n,model)
    Q = model.glbTrj;
    plot3(Q(3,1:t_n), ...                % Z
          Q(1,1:t_n), ...                % X
          zeros(1,length(Q(3,1:t_n))),'k:','LineWidth',1.5);   % Y
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