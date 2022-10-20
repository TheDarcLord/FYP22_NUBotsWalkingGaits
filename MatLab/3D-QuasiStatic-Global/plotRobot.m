function pass = plotRobot(index,model,params)
    % Origin
    a = 0.25;
    quiver3(0,0,0,a,0,0,"LineWidth",1,'Color','r');         % Z Vector
    quiver3(0,0,0,0,a,0,"LineWidth",1,'Color','#379203');   % X Vector
    quiver3(0,0,0,0,0,a,"LineWidth",1,'Color','b');         % Y Vector

    TOE  = [eye(3), [ 0.05; 0; 0];[0,0,0,1]];

    HTs   = kSLOW( model.q(:,index),index,model,params);
    
    rBER = HTs.ABER(1:3,4);                     % END EFFECTOR
    plot3(rBER(3),rBER(1),rBER(2), ...          %   RIGHT!
      'bo','LineWidth',1,'MarkerSize',5);
    ATFR = HTs.ABER*TOE;                        %
    rTFR = ATFR(1:3,4);                         % + TOE
    plot3([rBER(3),rTFR(3)],[rBER(1),rTFR(1)],[rBER(2),rTFR(2)],...
          'k','LineWidth',2);

    rBEL = HTs.ABEL(1:3,4);                     % END EFFECTOR
    plot3(rBEL(3),rBEL(1),rBEL(2), ...          %   LEFT!
          'ro','LineWidth',1,'MarkerSize',5);
    ATFL = HTs.ABEL*TOE;                        %
    rTFL = ATFL(1:3,4);                         % + TOE
    plot3([rBEL(3),rTFL(3)],[rBEL(1),rTFL(1)],[rBEL(2),rTFL(2)],...
          'k','LineWidth',2);

%% OTHER JOINTS & LINKS
    rLB0 = HTs.ALB0(1:3,4);                     % BASE LEFT
    plot3(rLB0(3),rLB0(1),rLB0(2),'rx','LineWidth',1,'markersize',5);

    plot3([rBEL(3), rLB0(3)],[rBEL(1), rLB0(1)],[rBEL(2), rLB0(2)],... % Link B-0
          'k','LineWidth',2);
    rRB0 = HTs.ARB0(1:3,4);                     % BASE RIGHT
    plot3(rRB0(3),rRB0(1),rRB0(2),'bx','LineWidth',1,'markersize',5);
    plot3([rBER(3), rRB0(3)],[rBER(1), rRB0(1)],[rBER(2), rRB0(2)],... % Link B-12
          'k','LineWidth',2);


    r01 = HTs.A01(1:3,4);                       % ONE
    plot3(r01(3),r01(1),r01(2),...              % -> Joint
          'rx','LineWidth',1,'markersize',5);
    plot3([rLB0(3),r01(3)],[rLB0(1),r01(1)],[rLB0(2),r01(2)],... % Link 0-1
          'k','LineWidth',2);
    r02 = HTs.A02(1:3,4);                       % TWO
    plot3(r02(3),r02(1),r02(2),...              % -> Joint
          'rx','LineWidth',1,'markersize',5); 
    plot3([r01(3),r02(3)],[r01(1),r02(1)],[r01(2),r02(2)],... % Link 1-2
          'k','LineWidth',2);
    r03 = HTs.A03(1:3,4);                       % THREE
    plot3(r03(3),r03(1),r03(2),...              % -> Joint
          'rx','LineWidth',1,'markersize',5);
    plot3([r02(3),r03(3)],[r02(1),r03(1)],[r02(2),r03(2)],... % Link 2-3
          'k','LineWidth',2);
    r04 = HTs.A04(1:3,4);                       % FOUR
    plot3(r04(3),r04(1),r04(2),...              % -> Joint
          'rx','LineWidth',1,'markersize',5);  
    plot3([r03(3),r04(3)],[r03(1),r04(1)],[r03(2),r04(2)],...
          'k','LineWidth',2);
    r05 = HTs.A05(1:3,4);                       % FIVE
    plot3(r05(3),r05(1),r05(2),...              % -> Joint
          'rx','LineWidth',1,'markersize',5);
    plot3([r04(3),r05(3)],[r04(1),r05(1)],[r04(2),r05(2)],...
          'k','LineWidth',2);
    r06 = HTs.A06(1:3,4);                       % SIX 
    plot3(r06(3),r06(1),r06(2),...              % -> Joint
          'bx','LineWidth',1,'markersize',5);    
    plot3([r05(3),r06(3)],[r05(1),r06(1)],[r05(2),r06(2)],...
          'k','LineWidth',2);
    r07 = HTs.A07(1:3,4);                       % SEVEN
    plot3(r07(3),r07(1),r07(2),...              % -> Joint
          'bx','LineWidth',1,'markersize',5);    
    plot3([r06(3),r07(3)],[r06(1),r07(1)],[r06(2),r07(2)],...
          'k','LineWidth',2);
    r08 = HTs.A08(1:3,4);                       % EIGHT
    plot3(r08(3),r08(1),r08(2),...              % -> Joint
          'bx','LineWidth',1,'markersize',5);
    plot3([r07(3),r08(3)],[r07(1),r08(1)],[r07(2),r08(2)],...
          'k','LineWidth',2);
    r09 = HTs.A09(1:3,4);                       % NINE
    plot3(r09(3),r09(1),r09(2),...              % -> Joint
          'bx','LineWidth',1,'markersize',5);
    plot3([r08(3),r09(3)],[r08(1),r09(1)],[r08(2),r09(2)],...
          'k', 'LineWidth',2);
    r010 = HTs.A010(1:3,4);                     % TEN
    plot3(r010(3),r010(1),r010(2),...           % -> Joint
          'bx','LineWidth',1,'markersize',5);         
    plot3([r09(3),r010(3)],[r09(1),r010(1)],[r09(2),r010(2)],...
          'k', 'LineWidth',2);
    r011 = HTs.A011(1:3,4);                     % ELEVEN
    plot3(r011(3),r011(1),r011(2),...           % -> Joint
          'bx','LineWidth',1,'markersize',5);
    plot3([r010(3),r011(3)],[r010(1),r011(1)],[r010(2),r011(2)],...
          'k', 'LineWidth',2);
    

    rCoM = model.r0CoMg(:,index);
    plot3(rCoM(3),rCoM(1),rCoM(2),'mo','MarkerSize',10,'LineWidth',2);
    drawnow
    pass = 1;
end