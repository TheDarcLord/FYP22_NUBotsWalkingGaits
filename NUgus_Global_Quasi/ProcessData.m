close all
load 40Hz_sinusoid.mat

TS = 1:1001;
INS = 1;

% 
% subtitle({"System Frequency: 40 {\itHz}", ...
%           "Time Instant: " + 0 + "{\it sec}"})
%%
TRAJECTORY = figure(1);
    subplot(1,2,1)
        title("Quasistatic: End Effector Trajectory",'FontSize',16);
        subtitle("System Frequency: 40 {\itHz}")
        hold on
        grid on
        grid minor
        axis equal
        view(90,90)
        axis([-0.2 0.6 -0.1 0.7 0 1])
        xlabel('Z Displacement ({\it metres})','FontSize',12,'FontWeight','bold')
        ylabel('X Displacement ({\it metres})','FontSize',12,'FontWeight','bold')
        zlabel('Y Displacement ({\it metres})','FontSize',12,'FontWeight','bold')
        plot3(model.glbTrj(3,TS),model.glbTrj(1,TS),model.glbTrj(2,TS)*0,'k','LineWidth',2)
        plot3(model.xe(3,TS),model.xe(1,TS),model.xe(2,TS),'color','#00B000','LineWidth',1.5)
        plot3(model.r0Lg(3,INS),model.r0Lg(1,INS),model.r0Lg(2,INS),'ro','MarkerSize',10,'LineWidth',3)
        plot3(model.r0Rg(3,INS),model.r0Rg(1,INS),model.r0Rg(2,INS),'bo','MarkerSize',10,'LineWidth',3)
        legend({'Trajectory {\bfQ}(t)','End Effector Trajectory','End Effector Left','End Effector Right'},'FontSize',12)
    subplot(1,2,2)
        title("Quasistatic: Foot Trajectories",'FontSize',16);
        subtitle("System Frequency: 40 {\itHz}")
        hold on
        grid on
        grid minor
        axis equal
        view(45,35)
        axis([-0.2 0.6 -0.1 0.7 0 0.2])
        xlabel('Z Displacement ({\it metres})','FontSize',12,'FontWeight','bold')
        ylabel('X Displacement ({\it metres})','FontSize',12,'FontWeight','bold')
        zlabel('Y Displacement ({\it metres})','FontSize',12,'FontWeight','bold')
        plot3(model.glbTrj(3,TS),model.glbTrj(1,TS),model.glbTrj(2,TS)*0,'k','LineWidth',2)
        plot3(model.r0Lg(3,TS),model.r0Lg(1,TS),model.r0Lg(2,TS),'r','MarkerSize',10,'LineWidth',1.5)
        plot3(model.r0Rg(3,TS),model.r0Rg(1,TS),model.r0Rg(2,TS),'b','MarkerSize',10,'LineWidth',1.5)
        legend({'Trajectory {\bfQ}(t)','End Effector Left','End Effector Right'},'FontSize',12)

%%
ROBOTINIT = figure(2);
    hold on
    grid on
    grid("minor")
    axis equal
    view(145,35)
    axis([-0.2 0.6 -0.1 0.7 0 0.6])
    title("Quasistatic: Initalised",'FontSize',16);
    subtitle("System Frequency: 40 {\itHz}")
    xlabel('Z Displacement ({\it metres})','FontSize',12,'FontWeight','bold')
    ylabel('X Displacement ({\it metres})','FontSize',12,'FontWeight','bold')
    zlabel('Y Displacement ({\it metres})','FontSize',12,'FontWeight','bold')
    plot3(model.r0CoMg(3,INS),model.r0CoMg(1,INS),0,'mx','LineWidth',3,MarkerSize=10)
    plot3(model.glbTrj(3,TS),model.glbTrj(1,TS),model.glbTrj(2,TS)*0,'k','LineWidth',2)
    [~] = plotRobot(1,model,params);
    legend({'Floor Projected CoM','Trajectory {\bfQ}(t)','Right End Effector','Right Support Polygon',...
        '','','','Left End Effector','Left Support Polgyon'},...
        'AutoUpdate','off','FontSize',12);

%%
ROBOTFRAME = figure(3);
    STS = 800:1001;
    INS = 1001;
    hold on
    grid on
    grid("minor")
    axis equal
    view(-140,15)
    axis([-0.2 0.6 -0.1 0.7 0 0.45])
    title("Quasistatic Locomotion",'FontSize',16);
    subtitle({"System Frequency: 40 {\itHz}", ...
          "Time Instant: " + model.tspan(STS(1)) + " to " + model.tspan(STS(end)) + "{\it secs}"},'FontSize',12)
    xlabel('Z Displacement ({\it metres})','FontSize',12,'FontWeight','bold')
    ylabel('X Displacement ({\it metres})','FontSize',12,'FontWeight','bold')
    zlabel('Y Displacement ({\it metres})','FontSize',12,'FontWeight','bold')
    plot3(model.r0CoMg(3,STS),model.r0CoMg(1,STS),0*model.r0CoMg(2,STS),'m','LineWidth',2)
    plot3(model.xe(3,STS),model.xe(1,STS),model.xe(2,STS),'color','#228B22','LineWidth',2);
    plot3(model.glbTrj(3,TS),model.glbTrj(1,TS),model.glbTrj(2,TS)*0,'k--','LineWidth',2)
    [~] = plotRobot(INS,model,params);
    [~] = plotRobotFeet(STS(1),model,params);
    legend({'Floor Projected CoM','End Effector Path','Trajectory {\bfQ}(t)','Right End Effector','Right Support Polygon',...
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
    axis([-0.1 0.5 -0.1 0.7 0 0.45])
    title("Quasistatic Locomotion: Waist Orientation",'FontSize',16);
    subtitle({"System Frequency: 40 {\itHz}", ...
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
