function pass = plotSteps(model)
    Q = model.glbTrj(:,:);
    plot3(Q(3,:), ...                % Z
          Q(1,:), ...                % X
          zeros(1,length(Q)),'r:');   % Y
    % Z X Y 
    P = model.pREF(:,:);
    plot3(P(2,:), ...                % Z
          P(1,:), ...                % X
          zeros(1,length(P)),'rx');  % Y
%     % Z X Y 
%     plot3(model.pREF(2,:), ...                % Z
%           model.pREF(1,:), ...                % X
%           zeros(1,length(model.pREF)),'r:');  % Y

    drawnow
    pass = 1;
end