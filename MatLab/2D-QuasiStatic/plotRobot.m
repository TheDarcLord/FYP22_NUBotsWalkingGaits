function plotRobot(index,model,params)
    params.mode = model.mode(index);
    [~, HTs] = kSLOW(model.q(:,index), index, model, params);

%     % ZERO: Z,X,Y,   Z,  X,  Y
%     quiver3(0,0,0, 0.5,0.0,0.0,'b','LineWidth',2); % Z
%     quiver3(0,0,0, 0.0,0.5,0.0,'r','LineWidth',2); % X
%     quiver3(0,0,0, 0.0,0.0,0.5,'g','LineWidth',2); % Y
    
    % MAIN COMPONENTS
    rBB = HTs.AbB(1:3,4);
    rB0 = HTs.Ab0(1:3,4);
    plot3([rBB(3), rB0(3)], ... Z
          [rBB(1), rB0(1)], ... X
          [rBB(2), rB0(2)], ... Y
        'k', 'LineWidth',2);
    plot3(rB0(3), rB0(1), rB0(2), 'rx', 'LineWidth',2,'MarkerSize',10);
    rB1 = HTs.Ab1(1:3,4);
    plot3([rB0(3) rB1(3)],[rB0(1) rB1(1)],[rB0(2) rB1(2)],...
        'k', 'LineWidth',2);
    plot3(rB1(3), rB1(1), rB1(2) ,'rx', 'LineWidth',2,'MarkerSize',10);
    rB2 = HTs.Ab2(1:3,4);
    plot3([rB1(3) rB2(3)], [rB1(1) rB2(1)], [rB1(2) rB2(2)],...
        'k', 'LineWidth',2);
    plot3(rB2(3), rB2(1), rB2(2) ,'rx', 'LineWidth',2,'MarkerSize',10);
    rbH = HTs.AbH(1:3,4);
    plot3(rbH(3), rbH(1), rbH(2), 'mx', 'LineWidth',2,'MarkerSize',10);
    rB3 = HTs.Ab3(1:3,4);
    plot3([rB2(3) rB3(3)], [rB2(1) rB3(1)], [rB2(2) rB3(2)],...
        'k', 'LineWidth',2);
    plot3(rB3(3), rB3(1), rB3(2), 'bx', 'LineWidth',2,'MarkerSize',10);
    rB4 = HTs.Ab4(1:3,4);
    plot3([rB3(3) rB4(3)], [rB3(1) rB4(1)], [rB3(2) rB4(2)],...
    'k', 'LineWidth',2);
    plot3(rB4(3), rB4(1), rB4(2), 'bx', 'LineWidth',2,'MarkerSize',10);
    rB5 = HTs.Ab5(1:3,4);
    plot3([rB4(3) rB5(3)], [rB4(1) rB5(1)], [rB4(2) rB5(2)],...
        'k', 'LineWidth',2);
    plot3(rB5(3), rB5(1), rB5(2), 'bx', 'LineWidth',2,'MarkerSize',10);
    rB6 = HTs.Ab6(1:3,4);
    plot3([rB5(3) rB6(3)], [rB5(1) rB6(1)], [rB5(2) rB6(2)],...
        'k', 'LineWidth',2);
    plot3(rB6(3), rB6(1), rB6(2), 'bx', 'LineWidth',2,'MarkerSize',10);
    rBE = HTs.AbE(1:3,4);
    plot3([rB6(3) rBE(3)], [rB6(1) rBE(1)], [rB6(2) rBE(2)],...
        'k', 'LineWidth',2);
    plot3(rBE(3), rBE(1), rBE(2), 'ko', 'LineWidth',2,'MarkerSize',10);
    axis([     -0.4,      0.2, ... % Z
           rbH(1)-1, rbH(1)+1, ... % X
                  0,        1]);   % Y
end