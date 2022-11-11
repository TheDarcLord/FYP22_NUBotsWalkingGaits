function DEBUG(j,t_begin,t_end,Qstep,model,params)
%DEBUG Plot certatin params at t = j
%   meh
        ROBOT_FRAME = figure(1);
        cla(ROBOT_FRAME);
        hold on
        grid on
        set(gca,'Color','#CCCCCC');
        title("3D Model - ZMP Walking",'FontSize',12);
        xlabel('{\bfZ} (metres)');
        ylabel('{\bfX} (metres)');
        zlabel('{\bfY} (metres)');
        view(-210,30);
        [~] = plotRobot(j,model,params);
        TIME = t_begin:t_end;
        plot3(Qstep(3,TIME),Qstep(1,TIME),Qstep(2,TIME),...
            'b-','LineWidth',2)
        drawnow
end