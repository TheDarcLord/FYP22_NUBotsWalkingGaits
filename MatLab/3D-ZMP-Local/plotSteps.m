function pass = plotSteps(IND,model)
    Q = model.glbTrj;
    plot3(Q(3,IND:end), ...                % Z
          Q(1,IND:end), ...                % X
          zeros(1,length(Q(3,IND:end))),'k--','LineWidth',1.5);   % Y
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