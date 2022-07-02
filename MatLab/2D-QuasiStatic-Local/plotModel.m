function [] = plotModel(HTs,model,i)
    hold on
    grid on
    view(55,35);

    set(gca, 'zdir','reverse');
    set(gca, 'ydir','reverse');
    set(gca,'Color','#EEEEEE');
    xlabel('{\bfX} (metres)');
    ylabel('{\bfY} (metres)');
    zlabel('{\bfZ} (metres)');
    axis([-1,1,-0.5,0.5,-1,0.05]);

    % ZERO: X,Y,Z    X, Y, Z
    quiver3(0,0,0, 0.5, 0, 0,'r','LineWidth',2); % X
    quiver3(0,0,0, 0, 0.5, 0,'b','LineWidth',2); % Y
    quiver3(0,0,0, 0, 0, 0.5,'g','LineWidth',2); % Z

    plot3(model.rGBg(1,i),model.rGBg(2,i),model.rGBg(3,i), ...  % BODY
          'mx','LineWidth',2,'MarkerSize',10);
    plot3(model.rCoMb(1,i),model.rCoMb(2,i),model.rCoMb(3,i), ... % CoM
          'ro','LineWidth',2,'MarkerSize',10);
    
    % LEFT
    rG0_L = HTs.AG0_L(1:3,4);
    plot3(rG0_L(1),rG0_L(2),rG0_L(3),'bx','LineWidth',2,'MarkerSize',5)
    plot3([model.rGBg(1,i) rG0_L(1)], ...
          [model.rGBg(2,i) rG0_L(2)], ...
          [model.rGBg(3,i) rG0_L(3)],'k', 'LineWidth',2);
    rG1_L = HTs.AG1_L(1:3,4);
    plot3(rG1_L(1),rG1_L(2),rG1_L(3),'bx','LineWidth',2,'MarkerSize',10)
    plot3([rG0_L(1) rG1_L(1)],[rG0_L(2) rG1_L(2)],[rG0_L(3) rG1_L(3)], ...
           'k', 'LineWidth',2);
    rG2_L = HTs.AG2_L(1:3,4);
    plot3(rG2_L(1),rG2_L(2),rG2_L(3),'bx','LineWidth',2,'MarkerSize',10)
    plot3([rG1_L(1) rG2_L(1)],[rG1_L(2) rG2_L(2)],[rG1_L(3) rG2_L(3)], ...
           'k', 'LineWidth',2);
    rG3_L = HTs.AG3_L(1:3,4);
    plot3(rG3_L(1),rG3_L(2),rG3_L(3),'bx','LineWidth',2,'MarkerSize',10)
    plot3([rG2_L(1) rG3_L(1)],[rG2_L(2) rG3_L(2)],[rG2_L(3) rG3_L(3)], ...
           'k', 'LineWidth',2);
    % RIGHT
    rG0_R = HTs.AG0_R(1:3,4);
    plot3(rG0_R(1),rG0_R(2),rG0_R(3),'bx','LineWidth',2,'MarkerSize',5)
    plot3([model.rGBg(1,i) rG0_R(1)], ...
          [model.rGBg(2,i) rG0_R(2)], ...
          [model.rGBg(3,i) rG0_R(3)],'k', 'LineWidth',2);
    rG1_R = HTs.AG1_R(1:3,4);
    plot3(rG1_R(1),rG1_R(2),rG1_R(3),'rx','LineWidth',2,'MarkerSize',10)
    plot3([rG0_R(1) rG1_R(1)],[rG0_R(2) rG1_R(2)],[rG0_R(3) rG1_R(3)], ...
           'k', 'LineWidth',2);
    rG2_R = HTs.AG2_R(1:3,4);
    plot3(rG2_R(1),rG2_R(2),rG2_R(3),'rx','LineWidth',2,'MarkerSize',10)
    plot3([rG1_R(1) rG2_R(1)],[rG1_R(2) rG2_R(2)],[rG1_R(3) rG2_R(3)], ...
           'k', 'LineWidth',2);
    rG3_R = HTs.AG3_R(1:3,4);
    plot3(rG3_R(1),rG3_R(2),rG3_R(3),'rx','LineWidth',2,'MarkerSize',10)
    plot3([rG2_R(1) rG3_R(1)],[rG2_R(2) rG3_R(2)],[rG2_R(3) rG3_R(3)], ...
           'k', 'LineWidth',2);

    legend({'+X_0','+Y_0','+Z_0','{\bfr}_B^{global}'},...
            Location='west');
end