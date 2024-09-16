% This Script plots all output of Simulation
% close all;
fontsize=24;
axiswidth=2;
graphwidth=2;
min_t=0;
max_t=10;
%% Voltage
figure(1)
V1=plot(out.V(:,1),out.V(:,2),'-');
hold on
V2=plot(out.V(:,1),out.V(:,3),'--');
V3=plot(out.V(:,1),out.V(:,4),':');
V4=plot(out.V(:,1),out.V(:,5),'-.');
V5=plot(out.V(:,1),out.V(:,6));
y=ylabel('\bf Voltages(V)');
x=xlabel('\bf Time(sec)'); 
l=legend({'\bf V_{DG1}','\bf V_{DG2}','\bf V_{DG3}','\bf V_{DG4}','\bf V_{MG}'},...
    'Location','southeast');
grid on
set(V1,'LineWidth',graphwidth);
set(V2,'LineWidth',graphwidth);
set(V3,'LineWidth',graphwidth);
set(V4,'LineWidth',graphwidth);
set(V5,'LineWidth',graphwidth);
set(x,'FontSize',fontsize,'FontName','Times New Roman');
set(y,'FontSize',fontsize,'FontName','Times New Roman');
set(l,'FontSize',20,'FontName','Times New Roman');
set(gca,'FontSize',fontsize, 'LineWidth', axiswidth,'fontweight','bold',...
    'xlim',[min_t max_t],'ylim',[47 48.5],'FontName','Times New Roman');
%% Average Voltage
figure(2)
Average=plot(out.Ave(:,1),out.Ave(:,2),'-');
y=ylabel('\bf Average Voltage(V)');
grid on
set(Average,'LineWidth',graphwidth);
% set(x,'FontSize',fontsize,'FontName','Times New Roman');
set(y,'FontSize',fontsize,'FontName','Times New Roman');
set(gca,'FontSize',fontsize, 'LineWidth', axiswidth,'fontweight','bold',...
    'xlim',[min_t max_t],'ylim',[47.2 49],'FontName','Times New Roman','xticklabel',[]);
hold on
