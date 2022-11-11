close all
load 200Hz_sinusoid.mat

TS = 1:1001;
INS = 1;

% 
% subtitle({"System Frequency: 200 {\itHz}", ...
%           "Time Instant: " + 0 + "{\it sec}"})
%%
TRAJECTORY = figure(1);
    subplot(1,2,1)
        title("Zero Moment Point: End Effector Trajectory",'FontSize',16);
        subtitle("System Frequency: 200 {\itHz}, Time Horizon: 1 {\itsec}")
        hold on
        grid on
        grid minor
        axis equal
        view(90,90)
        axis([-0.2 0.6 -0.1 0.8 0 1])
        xlabel('Z Displacement ({\it metres})','FontSize',12,'FontWeight','bold')
        ylabel('X Displacement ({\it metres})','FontSize',12,'FontWeight','bold')
        zlabel('Y Displacement ({\it metres})','FontSize',12,'FontWeight','bold')
        plot3(model.glbTrj(3,TS),model.glbTrj(1,TS),model.glbTrj(2,TS)*0,'k','LineWidth',2)
        plot3(model.r.xe(3,TS),model.r.xe(1,TS),model.r.xe(2,TS),'color','#00B000','LineWidth',1.5)
        plot3(model.r.r0Lg(3,INS),model.r.r0Lg(1,INS),model.r.r0Lg(2,INS),'ro','MarkerSize',10,'LineWidth',3)
        plot3(model.r.r0Rg(3,INS),model.r.r0Rg(1,INS),model.r.r0Rg(2,INS),'bo','MarkerSize',10,'LineWidth',3)
        legend({'Trajectory {\bfQ}(t)','End Effector Trajectory','End Effector Left','End Effector Right'},'FontSize',12)
    subplot(1,2,2)
        title("Zero Moment Point: Foot Trajectories",'FontSize',16);
        subtitle("System Frequency: 200 {\itHz}")
        hold on
        grid on
        grid minor
        axis equal
        view(45,35)
        axis([-0.2 0.6 -0.1 0.8 0 0.2])
        xlabel('Z Displacement ({\it metres})','FontSize',12,'FontWeight','bold')
        ylabel('X Displacement ({\it metres})','FontSize',12,'FontWeight','bold')
        zlabel('Y Displacement ({\it metres})','FontSize',12,'FontWeight','bold')
        plot3(model.glbTrj(3,TS),model.glbTrj(1,TS),model.glbTrj(2,TS)*0,'k','LineWidth',2)
        plot3(model.r.r0Lg(3,TS),model.r.r0Lg(1,TS),model.r.r0Lg(2,TS),'r','MarkerSize',10,'LineWidth',1.5)
        plot3(model.r.r0Rg(3,TS),model.r.r0Rg(1,TS),model.r.r0Rg(2,TS),'b','MarkerSize',10,'LineWidth',1.5)
        legend({'Trajectory {\bfQ}(t)','End Effector Left','End Effector Right'},'FontSize',12)

%%
ROBOTINIT = figure(2);
    hold on
    grid on
    grid("minor")
    axis equal
    view(145,25)
    axis([-0.2 0.6 -0.1 0.8 0 0.6])
    title("Zero Moment Point: Initalised",'FontSize',16);
    subtitle("System Frequency: 200 {\itHz}, Time Horizon: 1 {\itsec}")
    xlabel('Z Displacement ({\it metres})','FontSize',12,'FontWeight','bold')
    ylabel('X Displacement ({\it metres})','FontSize',12,'FontWeight','bold')
    zlabel('Y Displacement ({\it metres})','FontSize',12,'FontWeight','bold')
    plot3(model.glbTrj(3,TS),model.glbTrj(1,TS),model.glbTrj(2,TS)*0,'k','LineWidth',2);
    [~] = plotPend(1,model,params);
    [~] = plotRobot(1,model,params);
    legend({'Trajectory {\bfQ}(t)','{\bfZMP}^{REF}','Pendulum','','Right End Effector','Right Support Polygon',...
        '','','','Left End Effector','Left Support Polgyon'},...
        'AutoUpdate','off','FontSize',12);

%%
ROBOTFRAME = figure(3);
    STS = 848:972;
    INS = 972;
    hold on
    grid on
    grid("minor")
    axis equal
    view(-135,35)
    axis([-0.2 0.6 -0.1 0.8 0 0.45])
    title("Zero Moment Point Locomotion",'FontSize',16);
    subtitle({"System Frequency: 200 {\itHz}", ...
          "Time Instant: " + model.tspan(STS(1)) + " to " + model.tspan(STS(end)) + "{\it secs}"},'FontSize',12)
    xlabel('Z Displacement ({\it metres})','FontSize',12,'FontWeight','bold')
    ylabel('X Displacement ({\it metres})','FontSize',12,'FontWeight','bold')
    zlabel('Y Displacement ({\it metres})','FontSize',12,'FontWeight','bold')
    plot3(model.r.r0CoMg(3,100:STS(end)),model.r.r0CoMg(1,100:STS(end)),params.zc*model.r.r0CoMg(2,100:STS(end)).^0,'m','LineWidth',2)
    plot3(model.glbTrj(3,TS),model.glbTrj(1,TS),model.glbTrj(2,TS)*0,'k--','LineWidth',2)
    plot3(model.p.y(2,100:STS(end)), model.p.y(1,100:STS(end)), model.p.y(2,100:STS(end))*0,'color','#00A300','LineWidth',2)
    [~] = plotRobot(INS,model,params);
    [~] = plotRobotFeet(STS(1),model,params);
    legend({'Pendulum','End Effector Path','Achieved {\bfZMP}','Right End Effector','Right Support Polygon',...
            '','','','Left End Effector','Left Support Polgyon','','','','','','','','','','','','','','','','','','','','','','','','','','','','','','','','','','Former Right SP','','Former Left SP'},...
        'AutoUpdate','off','FontSize',12);

%%
ROBOTFRAMETWO = figure(4);
    INS = [1; 300; 500; 700; 1000];
    hold on
    grid on
    grid("minor")
    axis equal
    view(90,90)
    axis([-0.1 0.5 -0.1 0.8 0 0.45])
    title("Zero Moment Point Locomotion: Waist Orientation",'FontSize',16);
    subtitle({"System Frequency: 200 {\itHz}", ...
          "Time Instants: 0, " + ...
          model.tspan(INS(2)) + ", "     +...
          model.tspan(INS(3)) + ", "     +...
          model.tspan(INS(4)) + ",  & "  +...
          model.tspan(INS(5)) + " {\it secs}"},'FontSize',14)
    xlabel('Z Displacement ({\it metres})','FontSize',12,'FontWeight','bold')
    ylabel('X Displacement ({\it metres})','FontSize',12,'FontWeight','bold')
    zlabel('Y Displacement ({\it metres})','FontSize',12,'FontWeight','bold')
    plot3(model.glbTrj(3,TS),model.glbTrj(1,TS),model.glbTrj(2,TS)*0,'k--','LineWidth',2)
    for i=1:5
        [~] = plotRobotWaist(INS(i),model,params);
        legend({'Trajectory {\bfQ}(t)','Right Hip Joint','Left Hip Joint',...
                'Hip Vector'},...
            'AutoUpdate','off','FontSize',12);
    end

%%
TRAJECTORY = figure(5);
    subplot(2,1,1)
        title("Zero Moment Point X Trajectory",'FontSize',16);
        subtitle("System Frequency: 200 {\itHz}, Time Horizon: 1 {\itsec}")
        hold on
        grid on
        grid minor
        xlabel('Time ({\it seconds})','FontSize',12,'FontWeight','bold')
        ylabel('X Displacement ({\it metres})','FontSize',12,'FontWeight','bold')
        plot(model.tspan(TS),model.p.pREF(1,TS),'k-','LineWidth',2);
        plot(model.tspan(TS),model.p.y(1,TS),'r-','LineWidth',2);
        plot(model.tspan(TS),abs(model.p.pREF(1,TS) - model.p.y(1,TS)),'LineWidth',2,'Color','#0064FF');
        legend({'ZMP_{x}^{REF}','ZMP_{x}','Error'},'FontSize',12)
    subplot(2,1,2)
        title("Zero Moment Point Z Trajectory",'FontSize',16);
        subtitle("System Frequency: 200 {\itHz}, Time Horizon: 1 {\itsec}")
        hold on
        grid on
        grid minor
        xlabel('Time ({\it seconds})','FontSize',12,'FontWeight','bold')
        ylabel('X Displacement ({\it metres})','FontSize',12,'FontWeight','bold')
        plot(model.tspan(TS),model.p.pREF(2,TS),'k-','LineWidth',2);
        plot(model.tspan(TS),model.p.y(2,TS),'r-','LineWidth',2);
        plot(model.tspan(TS),abs(model.p.pREF(2,TS) - model.p.y(2,TS)),'LineWidth',2,'Color','#0064FF');
        legend({'ZMP_{z}^{REF}','ZMP_{z}','Error'},'FontSize',12)