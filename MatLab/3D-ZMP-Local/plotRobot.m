function pass = plotRobot(index,model,params)
    % Origin
    quiver3(0,0,0,1,0,0,"LineWidth",2,'Color','r');         % Z Vector
    quiver3(0,0,0,0,1,0,"LineWidth",2,'Color','#379203');   % X Vector
    quiver3(0,0,0,0,0,1,"LineWidth",2,'Color','b');         % Y Vector

    AFOOT = [eye(3), [0; 0; 0.05];[0,0,0,1]];    % Foot Transform

    HTs   = kSLOW( model.r.q(:,index), params);
    
%% OTHER JOINTS & LINKS
    rRB0 = HTs.ARB0(1:3,4);                     % BASE RIGHT
    plot3(rRB0(3),rRB0(1),rRB0(2),'bx','LineWidth',1,'markersize',5);
    rGEL = HTs.ARB0(1:3,4);                     % END EFFECTOR
    AGFL = HTs.ARB0*AFOOT;                      % + FOOT
    rGFL = AGFL(1:3,4);                         % + TOE
    plot3(rGEL(3),rGEL(1),rGEL(2), ...          %   RIGHT!
          'bo','LineWidth',1,'MarkerSize',5);
    plot3(rGFL(3),rGFL(1),rGFL(2), ...          %   RIGHT TOE!
          'bx','LineWidth',1,'MarkerSize',5);
    plot3([rGEL(3),rGFL(3)],[rGEL(1),rGFL(1)],[rGEL(2),rGFL(2)],...
          'k','LineWidth',2);

    rLB0 = HTs.ALB0(1:3,4);                     % BASE LEFT
    plot3(rLB0(3),rLB0(1),rLB0(2),'rx','LineWidth',1,'markersize',5);
    rGER = HTs.ALB0(1:3,4);                     % END EFFECTOR
    AGFR = HTs.ALB0*AFOOT;                      % + FOOT
    rGFR = AGFR(1:3,4);                         % + TOE
    plot3(rGER(3),rGER(1),rGER(2), ...          %   LEFT!
          'ro','LineWidth',1,'MarkerSize',5);
    plot3(rGFR(3),rGFR(1),rGFR(2), ...          %   LEFT TOE!
          'rx','LineWidth',1,'MarkerSize',5);
    plot3([rGER(3),rGFR(3)],[rGER(1),rGFR(1)],[rGER(2),rGFR(2)],...
          'k','LineWidth',2);


    

    r01 = HTs.A01(1:3,4);                       % ONE
    plot3(r01(3),r01(1),r01(2),...              % -> Joint
          'bx','LineWidth',1,'markersize',5);
    plot3([rRB0(3),r01(3)],[rRB0(1),r01(1)],[rRB0(2),r01(2)],... % Link 0-1
          'k','LineWidth',2);
    r02 = HTs.A02(1:3,4);                       % TWO
    plot3(r02(3),r02(1),r02(2),...              % -> Joint
          'bx','LineWidth',1,'markersize',5); 
    plot3([r01(3),r02(3)],[r01(1),r02(1)],[r01(2),r02(2)],... % Link 1-2
          'k','LineWidth',2);
    r03 = HTs.A03(1:3,4);                       % THREE
    plot3(r03(3),r03(1),r03(2),...              % -> Joint
          'bx','LineWidth',1,'markersize',5);
    plot3([r02(3),r03(3)],[r02(1),r03(1)],[r02(2),r03(2)],... % Link 2-3
          'k','LineWidth',2);
    r04 = HTs.A04(1:3,4);                       % FOUR
    plot3(r04(3),r04(1),r04(2),...              % -> Joint
          'bx','LineWidth',1,'markersize',5);  
    plot3([r03(3),r04(3)],[r03(1),r04(1)],[r03(2),r04(2)],...
          'k','LineWidth',2);
    r05 = HTs.A05(1:3,4);                       % FIVE
    plot3(r05(3),r05(1),r05(2),...              % -> Joint
          'bx','LineWidth',1,'markersize',5);
    plot3([r04(3),r05(3)],[r04(1),r05(1)],[r04(2),r05(2)],...
          'k','LineWidth',2);
    r06 = HTs.A06(1:3,4);                       % SIX 
    plot3(r06(3),r06(1),r06(2),...              % -> Joint
          'rx','LineWidth',1,'markersize',5);    
    plot3([r05(3),r06(3)],[r05(1),r06(1)],[r05(2),r06(2)],...
          'k','LineWidth',2);
    r07 = HTs.A07(1:3,4);                       % SEVEN
    plot3(r07(3),r07(1),r07(2),...              % -> Joint
          'rx','LineWidth',1,'markersize',5);    
    plot3([r06(3),r07(3)],[r06(1),r07(1)],[r06(2),r07(2)],...
          'k','LineWidth',2);
    r08 = HTs.A08(1:3,4);                       % EIGHT
    plot3(r08(3),r08(1),r08(2),...              % -> Joint
          'rx','LineWidth',1,'markersize',5);
    plot3([r07(3),r08(3)],[r07(1),r08(1)],[r07(2),r08(2)],...
          'k','LineWidth',2);
    r09 = HTs.A09(1:3,4);                       % NINE
    plot3(r09(3),r09(1),r09(2),...              % -> Joint
          'rx','LineWidth',1,'markersize',5);
    plot3([r08(3),r09(3)],[r08(1),r09(1)],[r08(2),r09(2)],...
          'k', 'LineWidth',2);
    r010 = HTs.A010(1:3,4);                     % TEN
    plot3(r010(3),r010(1),r010(2),...           % -> Joint
          'rx','LineWidth',1,'markersize',5);         
    plot3([r09(3),r010(3)],[r09(1),r010(1)],[r09(2),r010(2)],...
          'k', 'LineWidth',2);
    r011 = HTs.A011(1:3,4);                     % ELEVEN
    plot3(r011(3),r011(1),r011(2),...           % -> Joint
          'rx','LineWidth',1,'markersize',5);
    plot3([r010(3),r011(3)],[r010(1),r011(1)],[r010(2),r011(2)],...
          'k', 'LineWidth',2);
    

    rCoM = model.r.r0CoMg(:,index);
    plot3(rCoM(3),rCoM(1),rCoM(2),'mo','MarkerSize',10,'LineWidth',2);
    drawnow
    pass = 1;
end