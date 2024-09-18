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
l=legend({'\bf V_{DG1}','\bf V_{DG2}','\bf V_{DG3}','\bf V_{DG4}','\bf V_{ref}'},...
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
    'xlim',[min_t max_t],'ylim',[46.5 49.5],'FontName','Times New Roman');
%% DG Current
figure(2)
I1=plot(out.I(:,1),out.I(:,2),'-');
hold on
I2=plot(out.I(:,1),out.I(:,3),'--');
I3=plot(out.I(:,1),out.I(:,4),':');
I4=plot(out.I(:,1),out.I(:,5),'-.');
y=ylabel('\bf DG Currents(A)');
x=xlabel('\bf Time(sec)'); 
l=legend('\bf I_{DG1}','\bf I_{DG2}','\bf I_{DG3}','\bf I_{DG4}',...
    'Location','southeast');
grid on
set(I1,'LineWidth',graphwidth);
set(I2,'LineWidth',graphwidth);
set(I3,'LineWidth',graphwidth);
set(I4,'LineWidth',graphwidth);
set(x,'FontSize',fontsize,'FontName','Times New Roman');
set(y,'FontSize',fontsize,'FontName','Times New Roman');
set(l,'FontSize',20,'FontName','Times New Roman');
set(gca,'FontSize',fontsize, 'LineWidth', axiswidth,'fontweight','bold',...
    'xlim',[min_t max_t],'ylim',[10 100], 'FontName','Times New Roman');
%% Line Current
figure(3)
i1=plot(out.i(:,1),out.i(:,2),'-');
hold on
i2=plot(out.i(:,1),out.i(:,3),'--');
i3=plot(out.i(:,1),out.i(:,4),':');
i4=plot(out.i(:,1),out.i(:,5),'-.');
y=ylabel('\bf Line Currents(p.u.)');
x=xlabel('\bf Time(sec)'); 
l=legend('\bf i_{DG1}','\bf i_{DG2}','\bf i_{DG3}','\bf i_{DG4}',...
    'Location','southeast');
grid on
set(i1,'LineWidth',graphwidth);
set(i2,'LineWidth',graphwidth);
set(i3,'LineWidth',graphwidth);
set(i4,'LineWidth',graphwidth);
set(x,'FontSize',48,'FontName','Times New Roman');
set(y,'FontSize',fontsize,'FontName','Times New Roman');
set(l,'FontSize',20,'FontName','Times New Roman');
set(gca,'FontSize',fontsize, 'LineWidth', axiswidth,'fontweight','bold',...
    'xlim',[min_t max_t],'ylim',[-0.35 0.35],'FontName','Times New Roman');
%% Per-unit Current - Current Sharing
figure(4)
p1=plot(out.P(:,1),out.P(:,2),'-');
hold on
p2=plot(out.P(:,1),out.P(:,3),'--');
p3=plot(out.P(:,1),out.P(:,4),':');
p4=plot(out.P(:,1),out.P(:,5),'-.');
y=ylabel('\bf Currents(p.u.)');
x=xlabel('\bf Time(sec)'); 
l=legend('\bf i_{DG1}','\bf i_{DG2}','\bf i_{DG3}','\bf i_{DG4}',...
    'Location','southeast');
grid on
set(p1,'LineWidth',graphwidth);
set(p2,'LineWidth',graphwidth);
set(p3,'LineWidth',graphwidth);
set(p4,'LineWidth',graphwidth);
set(x,'FontSize',48,'FontName','Times New Roman');
set(y,'FontSize',fontsize,'FontName','Times New Roman');
set(l,'FontSize',20,'FontName','Times New Roman');
set(gca,'FontSize',fontsize, 'LineWidth', axiswidth,'fontweight','bold',...
    'xlim',[min_t max_t],'ylim',[0.9 1.1],'FontName','Times New Roman');
%% Average Voltage
figure(5)
Average=plot(out.Ave(:,1),out.Ave(:,2),'-');
y=ylabel('\bf Average Voltage(V)');
x=xlabel('\bf Time(sec)'); 
grid on
set(Average,'LineWidth',graphwidth);
% set(x,'FontSize',fontsize,'FontName','Times New Roman');
set(y,'FontSize',fontsize,'FontName','Times New Roman');
set(x,'FontSize',fontsize,'FontName','Times New Roman');
set(gca,'FontSize',fontsize, 'LineWidth', axiswidth,'fontweight','bold',...
    'xlim',[min_t max_t],'ylim',[46 50],'FontName','Times New Roman','xticklabel',[]);
hold on
