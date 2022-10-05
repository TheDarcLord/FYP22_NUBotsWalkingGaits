function pass = plotSteps(model)
    Q = model.glbTrj(:,:);
    plot3(Q(3,:), ...                % Z
          Q(1,:), ...                % X
          zeros(1,length(Q)),'r:');   % Y
    % Z X Y 
    P = model.p.pREF(:,:);
    plot3(P(2,:), ...                % Z
          P(1,:), ...                % X
          zeros(1,length(P)),'rx');  % Y
%     % Z X Y 
%     plot3(model.p.pREF(2,:), ...                % Z
%           model.p.pREF(1,:), ...                % X
%           zeros(1,length(model.p.pREF)),'r:');  % Y

    drawnow
    pass = 1;
end