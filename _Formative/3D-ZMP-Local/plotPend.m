function pass = plotPend(index,model,params)
    plot3(model.p.y(2,index),... % Y|Z
          model.p.y(1,index),... % X
          0,'kx','LineWidth',3,'MarkerSize',10)
    plot3(model.p.x(4,index),...    % Y|Z
          model.p.x(1,index),...    % X
          params.zc,'mo','LineWidth',3,'MarkerSize',10)
    plot3(model.p.y(2,1:index),...  % Y|Z
          model.p.y(1,1:index),...  % X
          zeros(1,length(model.tspan(1:index))),'k:','LineWidth',1)
    plot3(model.p.x(4,1:index),...  % Y|Z
          model.p.x(1,1:index),...  % X
          params.zc*ones(1,length(model.tspan(1:index))),...
          'm:','LineWidth',1)
    plot3([model.p.y(2,index) model.p.x(4,index)],...  % Y|Z
          [model.p.y(1,index) model.p.x(1,index)],...  % X
          [0 params.zc],'m-','LineWidth',1)
    
    drawnow
    pass = 1;
end