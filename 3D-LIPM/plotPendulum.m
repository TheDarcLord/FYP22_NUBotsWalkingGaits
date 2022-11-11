function plotPendulum(INDEX,model,params)
    hold on 
    grid on
    
    axis equal
    quiver3(0,0,0,0.25,0,0,'Color','b',...
        'LineWidth',2);    % Z vec
    quiver3(0,0,0,0,0.25,0,'Color','r',...
        'LineWidth',2);    % X vec
    quiver3(0,0,0,0,0,0.25,'Color','#00CA00',...
        'LineWidth',2);    % Y vec
    axis([-0.4 0.4 -0.1 2.1 0 0.6])
    
    plot3(model.glbTrj(3,:),model.glbTrj(1,:),model.glbTrj(2,:),...
        'k','LineWidth',2)
    
    plot3(model.x(4,INDEX),model.x(1,INDEX),params.zc,...
        'ro','LineWidth',3,'MarkerSize',10);

    plot3([model.y(2,INDEX) model.x(4,INDEX)], ...
          [model.y(1,INDEX) model.x(1,INDEX)], ...
          [0 params.zc],...
        'r','LineWidth',1.5);

    if params.CP == 1
        % Constraint Plane - CoM Bound to this plane
            Z     = @(X,Y) params.kx*X + params.ky*Y + params.zc; 
            [X,Y] = meshgrid(-10:0.1:10);
            surf(X,Y,Z(X,Y), ...
                 "FaceColor","r","LineStyle","none","FaceAlpha",0.08);
    end

    plot3(model.y(2,INDEX),model.y(1,INDEX),0,'bx','LineWidth',3,'MarkerSize',15)

    xlabel('{\bfZ} (metres)','FontSize',16);
    ylabel('{\bfX} (metres)','FontSize',16);
    zlabel('{\bfY} (metres)','FontSize',16);
end

