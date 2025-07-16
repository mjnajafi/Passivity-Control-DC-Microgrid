% "In God We Trust"
% This Script plots all output of Simulation
% close all;
fontsize=24;
axiswidth=2;
graphwidth=2;
min_t=0;
max_t=10;
% %% voltage
% figure(1)
% V1=plot(out.V(:,1),out.V(:,2),'-');
% hold on
% V2=plot(out.V(:,1),out.V(:,3),'--');
% V3=plot(out.V(:,1),out.V(:,4),':');
% V4=plot(out.V(:,1),out.V(:,5),'-.');
% V5=plot(out.V(:,1),out.V(:,6));
% y=ylabel('\bf Voltages(V)');
% x=xlabel('\bf Time(sec)'); 
% l=legend({'\bf V_{DG1}','\bf V_{DG2}','\bf V_{DG3}','\bf V_{DG4}','\bf V_r'},...
%     'Location','northeast');
% grid on
% set(V1,'LineWidth',graphwidth);
% set(V2,'LineWidth',graphwidth);
% set(V3,'LineWidth',graphwidth);
% set(V4,'LineWidth',graphwidth);
% set(V5,'LineWidth',graphwidth);
% set(x,'FontSize',fontsize,'FontName','Times New Roman');
% set(y,'FontSize',fontsize,'FontName','Times New Roman');
% set(l,'FontSize',20,'FontName','Times New Roman');
% set(gca,'FontSize',fontsize, 'LineWidth', axiswidth,'fontweight','bold',...
%     'xlim',[min_t max_t],'ylim',[0 80],'FontName','Times New Roman');
% %% Current
% figure(2)
% C1=plot(out.I(:,1),out.I(:,2),'-');
% hold on
% C2=plot(out.I(:,1),out.I(:,3),'--');
% C3=plot(out.I(:,1),out.I(:,4),':');
% C4=plot(out.I(:,1),out.I(:,5),'-.');
% y=ylabel('\bf Currents(A)');
% x=xlabel('\bf Time(sec)'); 
% l=legend('\bf i_{DG1}','\bf i_{DG2}','\bf i_{DG3}','\bf i_{DG4}',...
%     'Location','northeast');
% grid on
% set(C1,'LineWidth',graphwidth);
% set(C2,'LineWidth',graphwidth);
% set(C3,'LineWidth',graphwidth);
% set(C4,'LineWidth',graphwidth);
% set(x,'FontSize',fontsize,'FontName','Times New Roman');
% set(y,'FontSize',fontsize,'FontName','Times New Roman');
% set(l,'FontSize',20,'FontName','Times New Roman');
% set(gca,'FontSize',fontsize, 'LineWidth', axiswidth,'fontweight','bold',...
%     'xlim',[min_t max_t],'FontName','Times New Roman');
% %% Per Unit of Current
% figure(3)
% p1=plot(out.i_hat(:,1),out.i_hat(:,2),'-');
% hold on
% p2=plot(out.i_hat(:,1),out.i_hat(:,3),'--');
% p3=plot(out.i_hat(:,1),out.i_hat(:,4),':');
% p4=plot(out.i_hat(:,1),out.i_hat(:,5),'-.');
% p5=plot(out.i_hat(:,1),out.i_hat(:,6));
% y=ylabel('\bf Currents(p.u.)');
% x=xlabel('\bf Time(sec)'); 
% l=legend('\bf I_{DG1}','\bf I_{DG2}','\bf I_{DG3}','\bf I_{DG4}','\bf I_s',...
%     'Location','northeast');
% grid on
% set(p1,'LineWidth',graphwidth);
% set(p2,'LineWidth',graphwidth);
% set(p3,'LineWidth',graphwidth);
% set(p4,'LineWidth',graphwidth);
% set(p5,'LineWidth',graphwidth);
% set(x,'FontSize',48,'FontName','Times New Roman');
% set(y,'FontSize',fontsize,'FontName','Times New Roman');
% set(l,'FontSize',20,'FontName','Times New Roman');
% set(gca,'FontSize',fontsize, 'LineWidth', axiswidth,'fontweight','bold',...
%     'xlim',[min_t max_t],'ylim',[0 1.4],'FontName','Times New Roman');
%% Voltage and per-unit current at the same time
% figure(1)
% subplot(211)
% 
% V1=plot(out.V(:,1), out.V(:,2), '-r'); 
% hold on;
% V2=plot(out.V(:,1), out.V(:,3), '--g'); 
% V3=plot(out.V(:,1), out.V(:,4), ':b'); 
% V4=plot(out.V(:,1), out.V(:,5), '-.m'); 
% V5=plot(out.V(:,1), out.V(:,6), '--c'); 
% % V6=plot(out.V(:,1), out.V(:,7), ':y'); 
% % V7=plot(out.V(:,1), out.V(:,8), '-k'); 
% y=ylabel('\bf Voltages(V)');
% % x=xlabel('\bf Time(sec)'); 
% % l=legend({'\bf DG1','\bf DG2','\bf DG3','\bf DG4','\bf DG5','\bf DG6'},...
% %     'Location','southeast','Orientation','horizontal');
% grid on
% title('\bf (a)')
% set(V1,'LineWidth',graphwidth);
% set(V2,'LineWidth',graphwidth);
% set(V3,'LineWidth',graphwidth);
% set(V4,'LineWidth',graphwidth);
% set(V5,'LineWidth',graphwidth);
% % set(V6,'LineWidth',graphwidth);
% % set(V7,'LineWidth',graphwidth);
% % set(x,'FontSize',fontsize,'FontName','Times New Roman');
% set(y,'FontSize',fontsize,'FontName','Times New Roman');
% % set(l,'FontSize',20,'FontName','Times New Roman');
% set(gca,'FontSize',fontsize, 'LineWidth', axiswidth,'fontweight','bold',...
%     'xlim',[min_t max_t],'ylim',[39 70],'FontName','Times New Roman','xticklabel',[]);
% 
% subplot(212) 
% 
% p1=plot(out.i_hat(:,1), out.i_hat(:,2), '-r'); 
% hold on;
% p2=plot(out.i_hat(:,1), out.i_hat(:,3), '--g'); 
% p3=plot(out.i_hat(:,1), out.i_hat(:,4), ':b'); 
% p4=plot(out.i_hat(:,1), out.i_hat(:,5), '-.m'); 
% % p5=plot(out.i_hat(:,1), out.i_hat(:,6), '--c'); 
% % p6=plot(out.i_hat(:,1), out.i_hat(:,7), ':y'); 
% y=ylabel('\bf Currents(p.u.)');
% x=xlabel('\bf Time(sec)'); 
% l=legend({'\bf DG1','\bf DG2','\bf DG3','\bf DG4','\bf DG5','\bf DG6'},...
%     'Location','southeast','Orientation','horizontal');
% grid on
% title('\bf (b)')
% set(p1,'LineWidth',graphwidth);
% set(p2,'LineWidth',graphwidth);
% set(p3,'LineWidth',graphwidth);
% set(p4,'LineWidth',graphwidth);
% % set(p5,'LineWidth',graphwidth);
% % set(p6,'LineWidth',graphwidth);
% set(x,'FontSize',fontsize,'FontName','Times New Roman');
% set(y,'FontSize',fontsize,'FontName','Times New Roman');
% set(l,'FontSize',30,'FontName','Times New Roman');
% set(gca,'FontSize',fontsize, 'LineWidth', axiswidth,'fontweight','bold',...
%     'xlim',[min_t max_t],'ylim',[0 1.1],'FontName','Times New Roman');
%% Average Voltage
% figure(4)
% V1=plot(out.Ave(:,1),out.Ave(:,2),'-');
% hold on
% % V2=plot(out.Ave(:,1),out.Ave(:,3),'-');
% y=ylabel('\bf Voltages(V)');
% x=xlabel('\bf Time(sec)'); 
% l=legend({'\bf V_{DG1}','\bf V_{DG2}','\bf V_{DG3}','\bf V_{DG4}','\bf V_r'},...
%     'Location','northeast');
% grid on
% set(V1,'LineWidth',graphwidth);
% % set(V2,'LineWidth',graphwidth);
% set(x,'FontSize',fontsize,'FontName','Times New Roman');
% set(y,'FontSize',fontsize,'FontName','Times New Roman');
% set(l,'FontSize',20,'FontName','Times New Roman');
% set(gca,'FontSize',fontsize, 'LineWidth', axiswidth,'fontweight','bold',...
%     'xlim',[min_t max_t],'ylim',[0 80],'FontName','Times New Roman');
%% Average Voltage and Current Sharing
figure(1)
subplot(211)

V1=plot(out.Ave(:,1), out.Ave(:,2), '-g'); 
hold on;
% V2=plot(out.V(:,1), out.V(:,3), '--g'); 
% V3=plot(out.V(:,1), out.V(:,4), ':b'); 
% V4=plot(out.V(:,1), out.V(:,5), '-.m'); 
% V5=plot(out.V(:,1), out.V(:,6), '--c'); 
% V6=plot(out.V(:,1), out.V(:,7), ':y'); 
% V7=plot(out.V(:,1), out.V(:,8), '-k'); 
y=ylabel('\bf Average Voltages(V)');
% x=xlabel('\bf Time(sec)'); 
% l=legend({'\bf DG1','\bf DG2','\bf DG3','\bf DG4','\bf DG5','\bf DG6'},...
%     'Location','southeast','Orientation','horizontal');
grid on
title('\bf (a)')
set(V1,'LineWidth',graphwidth);
% set(V2,'LineWidth',graphwidth);
% set(V3,'LineWidth',graphwidth);
% set(V4,'LineWidth',graphwidth);
% set(V5,'LineWidth',graphwidth);
% set(V6,'LineWidth',graphwidth);
% set(V7,'LineWidth',graphwidth);
% set(x,'FontSize',fontsize,'FontName','Times New Roman');
set(y,'FontSize',fontsize,'FontName','Times New Roman');
% set(l,'FontSize',20,'FontName','Times New Roman');
set(gca,'FontSize',fontsize, 'LineWidth', axiswidth,'fontweight','bold',...
    'xlim',[min_t max_t],'ylim',[39 65],'FontName','Times New Roman','xticklabel',[]);

subplot(212) 

p1=plot(out.i_hat(:,1), out.i_hat(:,2), '-g'); 
hold on;
% p2=plot(out.i_hat(:,1), out.i_hat(:,3), '--g'); 
% p3=plot(out.i_hat(:,1), out.i_hat(:,4), ':b'); 
% p4=plot(out.i_hat(:,1), out.i_hat(:,5), '-.m'); 
% p5=plot(out.i_hat(:,1), out.i_hat(:,6), '--c'); 
% p6=plot(out.i_hat(:,1), out.i_hat(:,7), ':y'); 
y=ylabel('\bf Currents(p.u.)');
x=xlabel('\bf Time(sec)'); 
l=legend({'\bf With Disturbance'},...
    'Location','southeast','Orientation','horizontal');
grid on
title('\bf (b)')
set(p1,'LineWidth',graphwidth);
% set(p2,'LineWidth',graphwidth);
% set(p3,'LineWidth',graphwidth);
% set(p4,'LineWidth',graphwidth);
% set(p5,'LineWidth',graphwidth);
% set(p6,'LineWidth',graphwidth);
set(x,'FontSize',fontsize,'FontName','Times New Roman');
set(y,'FontSize',fontsize,'FontName','Times New Roman');
set(l,'FontSize',30,'FontName','Times New Roman');
set(gca,'FontSize',fontsize, 'LineWidth', axiswidth,'fontweight','bold',...
    'xlim',[min_t max_t],'ylim',[0 1.1],'FontName','Times New Roman');