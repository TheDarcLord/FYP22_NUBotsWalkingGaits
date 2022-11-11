function pass = plotRobotFeet(index,model,params)
    HTs = kSlow( model.r.q(:,index), index, model, params);
    def = [0,0,0,1];
    rBER = HTs.ABER(1:3,4); % END EFFECTOR RIGHT
    rT1 = HTs.ABER * [eye(3), [+0.12915; 0; +0.054]; def];
    rT2 = HTs.ABER * [eye(3), [-0.08385; 0; +0.054]; def];
    rT3 = HTs.ABER * [eye(3), [-0.08385; 0; -0.076]; def];
    rT4 = HTs.ABER * [eye(3), [+0.12915; 0; -0.076]; def];

    plot3(rBER(3),rBER(1),rBER(2),'bo','LineWidth',1,'MarkerSize',5);
    plot3([rT1(3,4), rT2(3,4)],[rT1(1,4), rT2(1,4)],[rT1(2,4), rT2(2,4)],'b:','LineWidth',2);
    plot3([rT2(3,4), rT3(3,4)],[rT2(1,4), rT3(1,4)],[rT2(2,4), rT3(2,4)],'b:','LineWidth',2);
    plot3([rT3(3,4), rT4(3,4)],[rT3(1,4), rT4(1,4)],[rT3(2,4), rT4(2,4)],'b:','LineWidth',2);
    plot3([rT4(3,4), rT1(3,4)],[rT4(1,4), rT1(1,4)],[rT4(2,4), rT1(2,4)],'b:','LineWidth',2);

    rBEL = HTs.ABEL(1:3,4); % END EFFECTOR LEFT
    lT1 = HTs.ABEL * [eye(3), [+0.12915; 0; +0.076]; def];
    lT2 = HTs.ABEL * [eye(3), [-0.08385; 0; +0.076]; def];
    lT3 = HTs.ABEL * [eye(3), [-0.08385; 0; -0.054]; def];
    lT4 = HTs.ABEL * [eye(3), [+0.12915; 0; -0.054]; def];
    
    plot3(rBEL(3),rBEL(1),rBEL(2),'ro','LineWidth',1,'MarkerSize',5);
    plot3([lT1(3,4), lT2(3,4)],[lT1(1,4), lT2(1,4)],[lT1(2,4), lT2(2,4)],'r:','LineWidth',2);
    plot3([lT2(3,4), lT3(3,4)],[lT2(1,4), lT3(1,4)],[lT2(2,4), lT3(2,4)],'r:','LineWidth',2);
    plot3([lT3(3,4), lT4(3,4)],[lT3(1,4), lT4(1,4)],[lT3(2,4), lT4(2,4)],'r:','LineWidth',2);
    plot3([lT4(3,4), lT1(3,4)],[lT4(1,4), lT1(1,4)],[lT4(2,4), lT1(2,4)],'r:','LineWidth',2);
                                                      
    drawnow
    pass = 1;
end