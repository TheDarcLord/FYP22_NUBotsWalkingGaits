function pass = plotSteps(model)
    Q = model.glbTrj;
    plot3(Q(3,:), ...                % Z
          Q(1,:), ...                % X
          zeros(1,length(Q)),'k-','LineWidth',2);   % Y
%     % Z X Y 
%     plot3(model.p.pREF(2,:), ...                % Z
%           model.p.pREF(1,:), ...                % X
%           zeros(1,length(model.p.pREF)),'rx');  % Y
%     % Z X Y 
%     plot3(model.p.pREF(2,:), ...                % Z
%           model.p.pREF(1,:), ...                % X
%           zeros(1,length(model.p.pREF)),'r:');  % Y

    drawnow
    pass = 1;
end