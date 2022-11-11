function pass = plotRobot(index,model,params)
    % Origin
    a   = 0.25;
    def = [0,0,0,1];
    HTs = kSlow( model.q(:,index),index,model,params);
%     quiver3(0,0,0,a,0,0,"LineWidth",1,'Color','r');         % Z Vector
%     quiver3(0,0,0,0,a,0,"LineWidth",1,'Color','#379203');   % X Vector
%     quiver3(0,0,0,0,0,a,"LineWidth",1,'Color','b');         % Y Vector

    rBER = HTs.ABER(1:3,4); % END EFFECTOR RIGHT
    rT1 = HTs.ABER * [eye(3), [+0.12915; 0; +0.054]; def];
    rT2 = HTs.ABER * [eye(3), [-0.08385; 0; +0.054]; def];
    rT3 = HTs.ABER * [eye(3), [-0.08385; 0; -0.076]; def];
    rT4 = HTs.ABER * [eye(3), [+0.12915; 0; -0.076]; def];

    plot3(rBER(3),rBER(1),rBER(2),'bo','LineWidth',1,'MarkerSize',5);
    plot3([rT1(3,4), rT2(3,4)],[rT1(1,4), rT2(1,4)],[rT1(2,4), rT2(2,4)],'b','LineWidth',1.5);
    plot3([rT2(3,4), rT3(3,4)],[rT2(1,4), rT3(1,4)],[rT2(2,4), rT3(2,4)],'b','LineWidth',1.5);
    plot3([rT3(3,4), rT4(3,4)],[rT3(1,4), rT4(1,4)],[rT3(2,4), rT4(2,4)],'b','LineWidth',1.5);
    plot3([rT4(3,4), rT1(3,4)],[rT4(1,4), rT1(1,4)],[rT4(2,4), rT1(2,4)],'b','LineWidth',1.5);

    rBEL = HTs.ABEL(1:3,4); % END EFFECTOR LEFT
    lT1 = HTs.ABEL * [eye(3), [+0.12915; 0; +0.076]; def];
    lT2 = HTs.ABEL * [eye(3), [-0.08385; 0; +0.076]; def];
    lT3 = HTs.ABEL * [eye(3), [-0.08385; 0; -0.054]; def];
    lT4 = HTs.ABEL * [eye(3), [+0.12915; 0; -0.054]; def];
    
    plot3(rBEL(3),rBEL(1),rBEL(2),'ro','LineWidth',1,'MarkerSize',5);
    plot3([lT1(3,4), lT2(3,4)],[lT1(1,4), lT2(1,4)],[lT1(2,4), lT2(2,4)],'r','LineWidth',1.5);
    plot3([lT2(3,4), lT3(3,4)],[lT2(1,4), lT3(1,4)],[lT2(2,4), lT3(2,4)],'r','LineWidth',1.5);
    plot3([lT3(3,4), lT4(3,4)],[lT3(1,4), lT4(1,4)],[lT3(2,4), lT4(2,4)],'r','LineWidth',1.5);
    plot3([lT4(3,4), lT1(3,4)],[lT4(1,4), lT1(1,4)],[lT4(2,4), lT1(2,4)],'r','LineWidth',1.5);
                                                      

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

    drawnow
    pass = 1;
end