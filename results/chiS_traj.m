clear all, close all, clc

load chiS_winter_weat_uav-ugv_20180311.txt

fig = figure(1);
plot(chiS_winter_weat_uav_ugv_20180311, '--o', 'LineWidth', 2);
lab(1) = xlabel('$iteration$');
lab(2) = ylabel('$error$');
set(lab,'Interpreter','latex');
set(lab,'FontSize',35);
grid;

saveas(fig, 'chiS_traj.eps', 'epsc');