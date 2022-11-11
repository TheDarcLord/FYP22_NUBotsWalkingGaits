clear all
close all
clc

%% Setup
    TIME_FINAL = [30, 10, 2.5, 1];
    TIME_STEP  = 1/40;
    COLOUR = 'kmbr';
    STYLE = '----';

%% Trajectory Generation - Cubic Polynomial Splines
   
    TrajectoryTest = figure(1);
        hold on
        grid on
        axis equal
        set(gca,'Color','#DDDDDD');
        xlabel('{\bfX} (metres)','FontSize',16);
        ylabel('{\bfY} (metres)','FontSize',16);
        zlabel('{\bfZ} (metres)','FontSize',16);
        title({'3 Waypoint Trajectory, varying T_{final}',...
               'Elevation: 15^{o}'},'FontSize',18);
        view(45,15);
        axis([0 0.2 0 0.2 0 0.1])

        for i=1:length(TIME_FINAL)
            D   = diag(1:3,-1);
            % Inital Waypoint
            qi  = [0, 0;
                   0, 0;
                   0, 0];
            ti  = 0;
            tti = ti.^(0:3).';
            TI  = [tti, D*tti];
            % Final Waypoint
            qf  = [0.2, 0;
                   0.2, 0;
                   0,    0];
            tf  = TIME_FINAL(i);
            ttf = tf.^(0:3).';
            TF  = [ttf, D*ttf];
            % Mid Waypoint
            qm  = [(qi(1,1) + qf(1,1))/2, (qf(1,1)-qi(1,1))/(tf-ti);
                   (qi(2,1) + qf(2,1))/2, (qf(2,1)-qi(2,1))/(tf-ti);
                                     0.1, 0];
            tm  = (ti + tf) / 2;
            ttm = tm.^(0:3).';
            TM  = [ttm, D*ttm];
            % Evaluate Polynomial Coefficients
%INCORRECT  C   = [qi qm qf] / [TI TM TF];      
            C1  = [qi qm] / [TI TM];
            C2  = [qm qf] / [TM TF];
            % Evaluate Time and Positions       %
%INCORRECT  tspan   = ti:TIME_STEP:tf;          
%INCORRECT  tt      = tspan.^((0:3).');         
%INCORRECT  q       = C*tt;         
            ttt1    = ti:TIME_STEP:tm;
            T1      = ttt1.^((0:3).');
            ttt2    = tm:TIME_STEP:tf;
            T2      = ttt2.^((0:3).');
            q       = [C1*T1 C2*T2];

            plot3(q(1,:),q(2,:),q(3,:),'LineWidth',2,...
                'LineStyle',STYLE(i),'Color',COLOUR(i));
        end
        lgd=legend('T_{final} = 30 sec ','T_{final} = 10 sec',...
                   'T_{final} = 2.5 sec','T_{final} = 1 sec ',...
               'Location','eastoutside');
        lgd.FontSize = 14;

 %% Trajectory Generation - Quintic
    TrajectoryTest = figure(2);
        hold on
        grid on
        axis equal
        set(gca,'Color','#DDDDDD');
        xlabel('{\bfX} (metres)','FontSize',16);
        ylabel('{\bfY} (metres)','FontSize',16);
        zlabel('{\bfZ} (metres)','FontSize',16);
        title({'3 Waypoint Trajectory, varying T_{final}',...
               'Elevation: 15^{o}'},'FontSize',18);
        view(45,15);
        axis([0 0.2 0 0.2 0 0.1])

        for i=1:length(TIME_FINAL)
            D   = diag(1:5,-1);
            % Inital Waypoint
            qi  = [0, 0, 0;
                   0, 0, 0;
                   0, 0, 0];
            ti  = 0;
            tti = ti.^(0:5).';
            TI  = [tti, D*tti, D^2*tti];
            % Final Waypoint
            qf  = [0.2, 0, 0;
                   0.2, 0, 0;
                   0,   0, 0];
            tf  = TIME_FINAL(i);
            ttf = tf.^(0:5).';
            TF  = [ttf, D*ttf, D^2*ttf];
            % Mid Waypoint
            qm  = [(qi(1,1) + qf(1,1))/2, (qf(1,1)-qi(1,1))/(tf-ti), 0;
                   (qi(2,1) + qf(2,1))/2, (qf(2,1)-qi(2,1))/(tf-ti), 0;
                                     0.1, 0, 0];
            tm  = (ti + tf) / 2;
            ttm = tm.^(0:5).';
            TM  = [ttm, D*ttm, D^2*ttm];
            % Evaluate Polynomial Coefficients
            C1  = [qi qm] / [TI TM];
            C2  = [qm qf] / [TM TF];
            % Evaluate Time and Positions
            ttt1    = ti:TIME_STEP:tm;
            T1      = ttt1.^((0:5).');
            ttt2    = tm:TIME_STEP:tf;
            T2      = ttt2.^((0:5).');
            q       = [C1*T1 C2*T2];

            plot3(q(1,:),q(2,:),q(3,:),'LineWidth',2,...
                'LineStyle',STYLE(i),'Color',COLOUR(i));
        end
        lgd=legend('T_{final} = 30 sec ','T_{final} = 10 sec',...
                   'T_{final} = 2.5 sec','T_{final} = 1 sec ',...
               'Location','eastoutside');
        lgd.FontSize = 14;
        
%% Trajectory Generation - Continuous Tim
    
    TrajectoryTest = figure(3);
        hold on
        grid on
        axis equal
        set(gca,'Color','#DDDDDD');
        xlabel('{\bfX} (metres)','FontSize',16);
        ylabel('{\bfY} (metres)','FontSize',16);
        zlabel('{\bfZ} (metres)','FontSize',16);
        title({'3 Waypoint Trajectory, varying T_{final}',...
               'Elevation: 15^{o}'},'FontSize',18);
        view(45,15);

        for i=1:length(TIME_FINAL)
            ti  = 0;
            tf  = TIME_FINAL(i);
            td  = tf - ti;
            qi  = [0.0; 0.0;   0];
            qf  = [0.2; 0.2;   0];
            qm  = [(qi(1) + qf(1))/2;
                   (qi(2) + qf(1))/2;
                                 0.1];
            
            Ay  = 2*qm(3) / (ti + (td/2))^2;
            Vy  = Ay*(ti + (td/2));
        
            Q = @(t)  [qi(1) + ((qf(1)-qi(1)) / td).*t;
                       qi(2) + ((qf(2)-qi(2)) / td).*t;
                       Vy*t - Ay.*(t.^2)./2];
        
            tspan = ti:TIME_STEP:tf;
            P = Q(tspan);
        
            plot3(P(1,:),P(2,:),P(3,:),'LineWidth',2,...
                    'LineStyle',STYLE(i),'Color',COLOUR(i));
        end
        


