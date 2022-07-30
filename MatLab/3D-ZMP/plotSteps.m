function pass = plotSteps(FIGURE_NAME,model)
    [Q,~,~]  = trajGenGlobal(model.tspan);
    plot3(Q(2,:), ...                % Z
          Q(1,:), ...                % X
          zeros(1,length(Q)),'r');  % Y
    % Z X Y 
    plot3(model.p.pREF(2,:), ...                % Z
          model.p.pREF(1,:), ...                % X
          zeros(1,length(model.p.pREF)),'rx');  % Y
    % Z X Y 
    plot3(model.p.pREF(2,:), ...                % Z
          model.p.pREF(1,:), ...                % X
          zeros(1,length(model.p.pREF)),'r:');  % Y

    drawnow
    pass = 1;
end