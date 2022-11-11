%% RESEND q(t)
t_end = 741;
arms = [deg2rad(105);
        deg2rad(-10);
        deg2rad(-30);
        deg2rad(-30);
        deg2rad(10);
        deg2rad(105)];

webotsMod   = [-1;  % θ₁
                1;  % θ₂
                1;  % θ₃
                1;  % θ₄
               -1;  % θ₅
                1;  % θ₆
               -1;  % θ₇
                1;  % θ₈
               -1;  % θ₉
               -1;  % θ₁₀
               -1;  % θ₁₁
                1]; % θ₁₂
for i=1:5
    commJointValues("10.1.1.3","10013",...
    [zeros(18,1)]...
);
end
%%
for i=1:5
    commJointValues("10.1.1.3","10013",...
        [webotsMod.*model.q(:,1); arms]...
    );
end
%%
for i=1:t_end
        commJointValues("10.1.1.3","10013",...
        [webotsMod.*model.q(:,i); arms]...
    );
end
