% This code is used to solve a Concentration control problem for a fixed PI control parameter
% You must fill INPUT DATA section to get solution
%% Coded by
% Mohamed Mohamed El-Sayed Atyya
% mohamed.atyya94@eng-st.cu.edu.eg
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
close all; clear all; clc;
%% INPUT DATA
        %% TF Paramater
        %{
                             -tau s
                         e
        G(s) = -----------------
                       Ts  + 1
        tau=Vd/q
        T=Vm/q
        %}
        q=[0.5 0.9 1 1.1 2];
        Vd=1;
        Vm=1;
        %% PI Controller Parameter
        %{
                                   1
        Gc(s) = k ( 1 + ---- )
                                 Ti s
        %}
        k=0.5;
        Ti=1.1;
        %% Simulation Time
        Time = 20;
        %% Plotting Control
        % Plot = 0 --> No Plotting data    Plot = 1 --> Plotting data
        Plot=1;  
        %% Save figures control
        % Save = 0 --> No Saving figures    Save = 1 --> Saving figures
        % The figures will saved in running folder directory
        Save=0;
%% Open Loop TF
tau=Vd./q;
T=Vm./q;
for i=1:length(T)
    numG{i} = 1;
    denG{i} = [T(i) 1];
    G{i} = tf(numG{i},denG{i},'InputDelay',tau(i));
end
%% Controller TF
numGc = k*[Ti 1];
denGc = [Ti 0];
Gc = tf(numGc, denGc);
%% Closed Loop with Step Input
for i=1:length(T)
    G_closed_loop = feedback(G{i}*Gc,1);
    [c{i},t{i}]=step(G_closed_loop,Time);
    [cc{i},tc{i}]=step(Gc*(1-G_closed_loop),Time);
end
%% Plotting
if Plot == 1
        %% Step Response
        set(0,'defaultfigureposition',[0 50 1700 630]);
        figure(1)
        hold all;
        set(gcf,'color','w');
        for i=1:length(T)
            plot(t{i}, c{i},'linewidth',2);
            Legend{i}=['q = ' num2str(q(i))];
        end
        grid on;
        legend(Legend);
        xlabel('Time (sec)','fontsize',18)
        ylabel('c(t)','fontsize',18)
        title('Step Response','fontsize',18)
        %% Control Action
        figure(2)
        hold all;
        set(gcf,'color','w');
        for i=1:length(T)
            plot(tc{i}, cc{i},'linewidth',2);
        end
        grid on;
        legend(Legend);
        xlabel('Time (sec)','fontsize',18)
        ylabel('u(t)','fontsize',18)
        title('Control Action','fontsize',18)
end
%% Save figures
if Save ==1 && Plot ==1
    for S=1:2
        figure(S);
        saveas(gcf, [num2str(S) '.png']);
    end
end        