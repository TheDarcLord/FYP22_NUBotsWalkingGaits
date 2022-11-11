function pass = plotRobotWaist(index,model,params)
    % Origin
    HTs = kSlow( model.q(:,index),index,model,params);
%     quiver3(0,0,0,a,0,0,"LineWidth",1,'Color','r');         % Z Vector
%     quiver3(0,0,0,0,a,0,"LineWidth",1,'Color','#379203');   % X Vector
%     quiver3(0,0,0,0,0,a,"LineWidth",1,'Color','b');         % Y Vector

    r05 = HTs.A05(1:3,4);                       % FIVE
    plot3(r05(3),r05(1),0,...              % -> Joint
          'ro','LineWidth',2,'markersize',5);
    r06 = HTs.A06(1:3,4);                       % SIX 
    plot3(r06(3),r06(1),0,...              % -> Joint
          'bo','LineWidth',2,'markersize',5);    
    plot3([r05(3),r06(3)],[r05(1),r06(1)],[0,0],...
          'color','#00D100','LineWidth',2);

    drawnow
    pass = 1;
end