function pass = plotSteps(model,tspan)
    Q = model.glbTrj(:,tspan);
    plot3(Q(3,:), ...                                 % Z
          Q(1,:), ...                                 % X
          zeros(1,length(Q)),'k-','LineWidth',1.5);   % Y
    % ZMP_reference
    P = model.p.pREF(:,tspan);
    plot3(P(2,:),P(1,:),zeros(1,length(P)),'rx');
    plot3(P(2,:),P(1,:),zeros(1,length(P)),'r:');
    drawnow
    pass = 1;
end