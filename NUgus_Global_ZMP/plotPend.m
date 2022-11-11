function pass = plotPend(index,model,params)
    plot3(model.p.pREF(2,index),... % Y|Z
          model.p.pREF(1,index),... % X
          0,'color','#00A300','Marker','diamond','LineWidth',2,'MarkerSize',15)
    plot3(model.p.x(4,index),...    % Y|Z
          model.p.x(1,index),...    % X
          params.zc,'mo','LineWidth',3,'MarkerSize',10)
    plot3([model.p.y(2,index) model.p.x(4,index)],...  % Y|Z
          [model.p.y(1,index) model.p.x(1,index)],...  % X
          [0 params.zc],'m-','LineWidth',1.5)
    
    drawnow
    pass = 1;
end