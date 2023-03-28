%% Value function plot
dbstop if error
v=5;
w=2.5;
a=2;
N=100;
T=v/a;
t=linspace(0,sqrt(T),N).^2;
S=a/2*T^2;
figure(1),clf,hold on
x_=cos(linspace(0,2*pi,100));
y_=sin(linspace(0,2*pi,100));
for ti=t
    plot(x_*ti*w,S-a/2*(T-ti).^2+y_*ti*w,'Color','k')
end
V_=0:0.1:1;
NV=length(V_);
LI=[];
LT={};
C=parula;
for j=1:NV
    V=V_(j);
    col=C(1+floor((j-1)*256/NV),:);
    y=S-a/2*(T-t).^2;
    de=S-y;
    r=t*w+V;
    mi=(de+r>r(end)) & (r+r(end)>de);
    if any(mi)
        d=diff(y);
        d0=[0,d];
        d1=[d,0];
        r0=[0,r(1:end-1)];
        r1=[r(2:end),0];
        li=(d0+r0>r) & (r0+r>d0);
        e=zeros(1,N);
        e(li)=acos((d0(li).^2+r(li).^2-r0(li).^2)./r(li)./d0(li)/2);
        b=zeros(1,N);
        ri=[li(2:end),0]==1;
        b(ri)=acos((d1(ri).^2+r(ri).^2-r1(ri).^2)./r(ri)./d1(ri)/2);
        c=zeros(1,N);
        c(mi)=acos((de(mi).^2+r(mi).^2-r(end)^2)./r(mi)./de(mi)/2);
        d=max(acos((de(mi).^2+r(end)^2-r(mi).^2)/r(end)./de(mi)/2));
        for i=1:find(c>b,1,'first')
            p=linspace(e(i),pi-max(b(i),c(i)),30);
            plot(sin(p)*r(i),-cos(p)*r(i)+y(i),'Color',col);
            plot(-sin(p)*r(i),-cos(p)*r(i)+y(i),'Color',col)
        end
    else
        d=0;
    end
    p=linspace(d,2*pi-d,100);
    LI(j)=plot(sin(p)*r(end),-cos(p)*r(end)+S,'Color',col);
    LT{j}="V="+num2str(V);
end

plot([0,0],[0,S],'r')
plot(0,0,'r^')
plot(0,S,'rx')
xlabel('x [m]', 'FontSize', 8)
ylabel('y [m]', 'FontSize', 8)
l=legend(LI,LT, 'Location','NorthEast')
l.FontSize =8
axis equal
filename = [folder 'contour_5_25' '.fig'];
saveas(figure(1),[pwd  filename]);
filename = [folder 'contour_5_25' '.pdf'];
% filename = fullfile([pwd filename])
set(gcf, 'Units', 'Inches', 'Position', [0, 0, 3.5,3], 'PaperUnits', 'Inches', 'PaperSize', [3.5, 3])
exportgraphics(gcf, [pwd filename]);





%% time hist
folder = '/Figures/';
figure(1),clf,hold on
load APF_method
histogram(results(results>0),20,'facealpha',0.5)
apf_successful = sum(results>0);
apf_fastest = min(results(results>0));
l = legend({"APF, " +  num2str(apf_successful) + " runs"},'Location','NorthEast')
l.FontSize = 8;
xlabel('Elapsed time [s]', 'FontSize', 8)
ylabel('Rounds [-]', 'FontSize', 8)
filename = [folder 'Hist_APF' '.fig'];
saveas(figure(1),[pwd  filename]);
filename = [folder 'Hist_APF' '.pdf'];
% filename = fullfile([pwd filename])
set(gcf, 'Units', 'Inches', 'Position', [0, 0, 3.5,3], 'PaperUnits', 'Inches', 'PaperSize', [3.5, 3])
exportgraphics(gcf, [pwd filename]);
close all



load SVO_methodBangBang
histogram(results(results>0),20,'facealpha',0.5)
svo_successful = sum(results>0);
svo_fastest = min(results(results>0));
l = legend({"SVO, " + num2str(svo_successful) + " runs"}, 'Location','NorthEast')
l.FontSize = 8;
xlabel('Elapsed time [s]', 'FontSize', 8)
ylabel('Rounds [-]', 'FontSize', 8)
filename = [folder 'Hist_SVO' '.fig'];
saveas(figure(1),[pwd  filename]);
filename = [folder 'Hist_SVO' '.pdf'];
% filename = fullfile([pwd filename])
set(gcf, 'Units', 'Inches', 'Position', [0, 0, 3.5,3], 'PaperUnits', 'Inches', 'PaperSize', [3.5, 3])
exportgraphics(gcf, [pwd filename]);
close all



load EBG_method
histogram(results(results>0),20,'facealpha',0.5)
ebg_successful = sum(results>0);
ebg_fastest = min(results(results>0));
XL=xlim;
xlim([10 * floor(min(ebg_fastest,min(apf_fastest,svo_fastest))/10),XL(2)]) % min time=30.8 s
l = legend({"EBG, " + num2str(ebg_successful) + " runs"}, 'Location','NorthEast')
l.FontSize = 8;
xlabel('Elapsed time [s]', 'FontSize', 8)
ylabel('Rounds [-]', 'FontSize', 8)
filename = [folder 'Hist_EBG' '.fig'];
saveas(figure(1),[pwd  filename]);
filename = [folder 'Hist_EBG' '.pdf'];
% filename = fullfile([pwd filename])
set(gcf, 'Units', 'Inches', 'Position', [0, 0, 3.5,3], 'PaperUnits', 'Inches', 'PaperSize', [3.5, 3])
exportgraphics(gcf, [pwd filename]);
close all


%% heatmap plot
load APF_method
Nbin=50;
Dbins=linspace(0,10,Nbin+1);
Vbins=linspace(0,5,Nbin+1);
M=zeros(Nbin);
for i=1:runs
    if results(i)>0 % use data only from successful runs
        M = M+histcounts2(velocity_distance_data{i}(:,2:end), ...
            repmat(velocity_distance_data{i}(:,1),1,parameters.NP), Dbins, Vbins);
    end
end
figure(1)
imagesc(Vbins(1:end),Dbins,M(:,1:end))
set(gca,'ColorScale','log')
xlabel('Vehicle velocity [m/s]', 'FontSize', 8)
ylabel('Pedestrian distance [m]', 'FontSize', 8)
h = colorbar;
set(get(h,'label'),'string','Occurences over all simulations', 'FontSize', 8);
filename = [folder 'HeatMap_EBG' '.fig'];
saveas(figure(1),[pwd  filename]);
filename = [folder 'HeatMap_EBG' '.pdf'];
% filename = fullfile([pwd filename])
set(gcf, 'Units', 'Inches', 'Position', [0, 0, 3.5,3], 'PaperUnits', 'Inches', 'PaperSize', [3.5, 3])
exportgraphics(gcf, [pwd filename]);
