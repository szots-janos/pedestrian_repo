%% Final path plotting:
% plot of agents' trajectories
close all
clc

load('EBG_82');

toIndex = results/0.1;


for iNum=1:6
   close all
    figure(1)

load('EBG_82');

if iNum==6
   h(1) =  plot(squeeze(XE(1,1, 1:end   )),squeeze(XE(2,1,1:end  )), 'color','k', 'LineWidth', 2, 'LineStyle', '-');
   hold on
else
   h(1) = plot(squeeze(XE(1,1, 1:(iNum*floor(toIndex/10)))),squeeze(XE(2,1,1:(iNum*floor(toIndex/10))   )), 'color','k', 'LineWidth', 2, 'LineStyle', '-');
    hold on
end

% Plot the persuers:
for i=1:size(XP, 2)
    if iNum==6
    plot(squeeze(XP(1,i,(1:end) )), squeeze(XP(2,i,(1:end) )), 'Color', [mod(i,2) mod(i,3)/3 mod(i,4)/4]);
    hold on
    else
    plot(squeeze(XP(1,i,1:(iNum*floor(toIndex/10)) )), squeeze(XP(2,i,1:(iNum*floor(toIndex/10)) )), 'Color', [mod(i,2) mod(i,3)/3 mod(i,4)/4]);
    hold on
    end
end


load('SVO_82');
if (  (abs(results)/0.1)>=(iNum*floor(toIndex/10)) )
   h(2) = plot(squeeze(XE(1,1,1:(iNum*floor(toIndex/10)) )),squeeze(XE(2,1,1:(iNum*floor(toIndex/10)) )), 'color', 'b', 'LineWidth', 2, 'LineStyle', '-');
    hold on
else
   h(2) = plot(squeeze(XE(1,1,1:end )),squeeze(XE(2,1,1:end )), 'color', 'b', 'LineWidth', 2, 'LineStyle', '-');
    hold on
end


load('APF_82');

if (  (abs(results)/0.1)>=(iNum*floor(toIndex/10)) )
  h(3)=  plot(squeeze(XE(1,1,1:(iNum*floor(toIndex/10)) )),squeeze(XE(2,1,1:(iNum*floor(toIndex/10)) )), 'color', 'r', 'LineWidth', 2, 'LineStyle', '-');
    hold on
else
  h(3) =  plot(squeeze(XE(1,1,1:end )),squeeze(XE(2,1,1:end )), 'color', 'r', 'LineWidth', 2, 'LineStyle', '-');
    hold on
end


% plot(squeeze(XE(1,1,end)),squeeze(XE(2,1,end)),'.','color','k');
% for i=1:size(XP, 2)
%     plot(squeeze(XP(1,i,end)),squeeze(XP(2,i,end)),'.','color','k')
% end

plot(target(1), target(2), 'k^')
xlabel('x [m]', 'FontSize', 8)
ylabel('y [m]', 'FontSize', 8)
axis equal
l=legend(h, 'EBG', 'SVO', 'APF', 'location', 'southwest')
l.FontSize = 8;
folder = '/Figures/';
filename = [folder ['Motion_', num2str(iNum)] '.fig'];
saveas(figure(1),[pwd  filename]);
filename = [folder ['Motion_', num2str(iNum)] '.pdf'];
% filename = fullfile([pwd filename])
set(gcf, 'Units', 'Inches', 'Position', [0, 0, 3.5,3], 'PaperUnits', 'Inches', 'PaperSize', [3.5, 3])
exportgraphics(gcf, [pwd filename]);


end

