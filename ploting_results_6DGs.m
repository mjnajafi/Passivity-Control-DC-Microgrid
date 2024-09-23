% This Script plots all output of Simulation
close all;
fontsize=24;
axiswidth=2;
graphwidth=2;
min_t=0;
max_t=10;
%% Voltage
figure(1)
V1=plot(out.V(:,1), out.V(:,2), '-r'); 
hold on;
V2=plot(out.V(:,1), out.V(:,3), '--g'); 
V3=plot(out.V(:,1), out.V(:,4), ':b'); 
V4=plot(out.V(:,1), out.V(:,5), '-.m'); 
V5=plot(out.V(:,1), out.V(:,6), '--c'); 
V6=plot(out.V(:,1), out.V(:,7), ':y'); 
V7=plot(out.V(:,1), out.V(:,8), '-k'); 
y=ylabel('\bf Voltages(V)');
x=xlabel('\bf Time(sec)'); 
l=legend({'\bf V_{DG1}','\bf V_{DG2}','\bf V_{DG3}','\bf V_{DG4}','\bf V_{DG5}','\bf V_{DG6}','\bf V_{ref}'},...
    'Location','southeast');
grid on
set(V1,'LineWidth',graphwidth);
set(V2,'LineWidth',graphwidth);
set(V3,'LineWidth',graphwidth);
set(V4,'LineWidth',graphwidth);
set(V5,'LineWidth',graphwidth);
set(V6,'LineWidth',graphwidth);
set(V7,'LineWidth',graphwidth);
set(x,'FontSize',fontsize,'FontName','Times New Roman');
set(y,'FontSize',fontsize,'FontName','Times New Roman');
set(l,'FontSize',20,'FontName','Times New Roman');
set(gca,'FontSize',fontsize, 'LineWidth', axiswidth,'fontweight','bold',...
    'xlim',[min_t max_t],'ylim',[47 48.5],'FontName','Times New Roman');
%% DG Current
figure(2)
I1=plot(out.I(:,1), out.I(:,2), '-r'); 
hold on;
I2=plot(out.I(:,1), out.I(:,3), '--g'); 
I3=plot(out.I(:,1), out.I(:,4), ':b'); 
I4=plot(out.I(:,1), out.I(:,5), '-.m'); 
I5=plot(out.I(:,1), out.I(:,6), '--c'); 
I6=plot(out.I(:,1), out.I(:,7), ':y'); 
y=ylabel('\bf DG Currents(A)');
x=xlabel('\bf Time(sec)'); 
l=legend('\bf I_{DG1}','\bf I_{DG2}','\bf I_{DG3}','\bf I_{DG4}','\bf I_{DG5}','\bf I_{DG6}',...
    'Location','northeast');
grid on
set(I1,'LineWidth',graphwidth);
set(I2,'LineWidth',graphwidth);
set(I3,'LineWidth',graphwidth);
set(I4,'LineWidth',graphwidth);
set(I5,'LineWidth',graphwidth);
set(I6,'LineWidth',graphwidth);
set(x,'FontSize',fontsize,'FontName','Times New Roman');
set(y,'FontSize',fontsize,'FontName','Times New Roman');
set(l,'FontSize',20,'FontName','Times New Roman');
set(gca,'FontSize',fontsize, 'LineWidth', axiswidth,'fontweight','bold',...
    'xlim',[min_t max_t],'ylim',[10 80], 'FontName','Times New Roman');
%% Line Current
figure(3)
i1=plot(out.i(:,1), out.i(:,2), '-r'); 
hold on;
i2=plot(out.i(:,1), out.i(:,3), '--g'); 
i3=plot(out.i(:,1), out.i(:,4), ':b'); 
i4=plot(out.i(:,1), out.i(:,5), '-.m'); 
i5=plot(out.i(:,1), out.i(:,6), '--c'); 
i6=plot(out.i(:,1), out.i(:,7), ':y'); 
y=ylabel('\bf Line Currents(p.u.)');
x=xlabel('\bf Time(sec)'); 
l=legend('\bf i_{DG1}','\bf i_{DG2}','\bf i_{DG3}','\bf i_{DG4}','\bf i_{DG5}','\bf i_{DG46}',...
    'Location','southeast');
grid on
set(i1,'LineWidth',graphwidth);
set(i2,'LineWidth',graphwidth);
set(i3,'LineWidth',graphwidth);
set(i4,'LineWidth',graphwidth);
set(i5,'LineWidth',graphwidth);
set(i6,'LineWidth',graphwidth);
set(x,'FontSize',48,'FontName','Times New Roman');
set(y,'FontSize',fontsize,'FontName','Times New Roman');
set(l,'FontSize',20,'FontName','Times New Roman');
set(gca,'FontSize',fontsize, 'LineWidth', axiswidth,'fontweight','bold',...
    'xlim',[min_t max_t],'ylim',[-0.3 0.3],'FontName','Times New Roman');
%% Per-unit Current - Current Sharing
figure(4)
p1=plot(out.P(:,1), out.P(:,2), '-r'); 
hold on;
p2=plot(out.P(:,1), out.P(:,3), '--g'); 
p3=plot(out.P(:,1), out.P(:,4), ':b'); 
p4=plot(out.P(:,1), out.P(:,5), '-.m'); 
p5=plot(out.P(:,1), out.P(:,6), '--c'); 
p6=plot(out.P(:,1), out.P(:,7), ':y'); 
y=ylabel('\bf Currents(p.u.)');
x=xlabel('\bf Time(sec)'); 
l=legend('\bf i_{DG1}','\bf i_{DG2}','\bf i_{DG3}','\bf i_{DG4}','\bf i_{DG5}','\bf i_{DG6}',...
    'Location','southeast');
grid on
set(p1,'LineWidth',graphwidth);
set(p2,'LineWidth',graphwidth);
set(p3,'LineWidth',graphwidth);
set(p4,'LineWidth',graphwidth);
set(p5,'LineWidth',graphwidth);
set(p6,'LineWidth',graphwidth);
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
    'xlim',[min_t max_t],'ylim',[47.2 48.8],'FontName','Times New Roman','xticklabel',[]);
%% Voltage and per-unit current at the same time

figure(6)
subplot(211)

V1=plot(out.V(:,1), out.V(:,2), '-r'); 
hold on;
V2=plot(out.V(:,1), out.V(:,3), '--g'); 
V3=plot(out.V(:,1), out.V(:,4), ':b'); 
V4=plot(out.V(:,1), out.V(:,5), '-.m'); 
V5=plot(out.V(:,1), out.V(:,6), '--c'); 
V6=plot(out.V(:,1), out.V(:,7), ':y'); 
V7=plot(out.V(:,1), out.V(:,8), '-k'); 
y=ylabel('\bf Voltages(V)');
% x=xlabel('\bf Time(sec)'); 
% l=legend({'\bf DG1','\bf DG2','\bf DG3','\bf DG4','\bf DG5','\bf DG6'},...
%     'Location','southeast','Orientation','horizontal');
grid on
title('\bf (a)')
set(V1,'LineWidth',graphwidth);
set(V2,'LineWidth',graphwidth);
set(V3,'LineWidth',graphwidth);
set(V4,'LineWidth',graphwidth);
set(V5,'LineWidth',graphwidth);
set(V6,'LineWidth',graphwidth);
set(V7,'LineWidth',graphwidth);
% set(x,'FontSize',fontsize,'FontName','Times New Roman');
set(y,'FontSize',fontsize,'FontName','Times New Roman');
% set(l,'FontSize',20,'FontName','Times New Roman');
set(gca,'FontSize',fontsize, 'LineWidth', axiswidth,'fontweight','bold',...
    'xlim',[min_t max_t],'ylim',[46 50],'FontName','Times New Roman');

subplot(212) 

p1=plot(out.P(:,1), out.P(:,2), '-r'); 
hold on;
p2=plot(out.P(:,1), out.P(:,3), '--g'); 
p3=plot(out.P(:,1), out.P(:,4), ':b'); 
p4=plot(out.P(:,1), out.P(:,5), '-.m'); 
p5=plot(out.P(:,1), out.P(:,6), '--c'); 
p6=plot(out.P(:,1), out.P(:,7), ':y'); 
y=ylabel('\bf Currents(p.u.)');
x=xlabel('\bf Time(sec)'); 
l=legend({'\bf DG1','\bf DG2','\bf DG3','\bf DG4','\bf DG5','\bf DG6'},...
    'Location','southeast','Orientation','horizontal');
grid on
title('\bf (b)')
set(p1,'LineWidth',graphwidth);
set(p2,'LineWidth',graphwidth);
set(p3,'LineWidth',graphwidth);
set(p4,'LineWidth',graphwidth);
set(p5,'LineWidth',graphwidth);
set(p6,'LineWidth',graphwidth);
set(x,'FontSize',48,'FontName','Times New Roman');
set(y,'FontSize',fontsize,'FontName','Times New Roman');
set(l,'FontSize',20,'FontName','Times New Roman');
set(gca,'FontSize',fontsize, 'LineWidth', axiswidth,'fontweight','bold',...
    'xlim',[min_t max_t],'ylim',[0.9 1.1],'FontName','Times New Roman');