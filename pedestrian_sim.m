clc
clear all
dbstop if error


%% global params
random_seed = 4; % to test one specific setup - works only if runs==1
dt = .1;
dc = 2; % capture distance (= obstacle+vehicle radius)

% pursuer params
NP = 30; % number of pursuers
vP = 2; % maximum speed, m/s
rP = 1; % turning radius, m, only used in dubins model
aP = 2; % maximum acceleration, m/s2, only used in dubins model
p_model = @omni;
p_strategy = @P_random;
p_args.avg_time = 3; % P_random: average direction change interval

% evader params
vE = 5; % maximum speed, m/s
rE = 5; % turning radius, m, only used in dubins model
aE = 2; % maximum acceleration, m/s2, only used in dubins model
e_model = @dubins;
e_strategy = @E_apf;
e_args.target_p=0.01; % apf agent parameter
e_args.rho_mult=2; % apf agent parameter
e_args.accel_mult=2; % apf agent parameter
e_args.reaction_distance = dc * 10; % ebg parameter
e_args.focus_distance = dc * 4; % ebg parameter
e_args.safety_distance = dc * 2; % ebg parameter

% start params
min_dist = 10;
max_dist = 50;
start_distribution = 1; % 0: circular, 1: rectangle

% end params
target = [150;0];
Tmax = 200;

% run params
runs = 100;
max_time = 10000; % max runtime in seconds

% save - careful, we do not check for overwrite!
save_filename = 'APF_method'; % set to '' to skip saving. use single ' marks!

%% init
parameters.NP = NP;
parameters.NE = 1;
parameters.vP = vP;
parameters.rP = rP;
parameters.aP = aP;
parameters.vE = vE;
parameters.rE = rE;
parameters.aE = aE;
parameters.dt = dt;
parameters.dc = dc;
parameters.target = target;
parameters.runs = runs;

P = [];
for i=1:NP, p=p_strategy(p_model(parameters.vP, parameters.rP, parameters.aP),...
        parameters, i, p_args); P=[P,p]; end

E = e_strategy(e_model(parameters.vE,parameters.rE,parameters.aE), ...
    parameters, NP + 1, e_args);

%% run
results = nan(runs,1);
velocity_distance_data = cell(runs, 1);
starttime=tic;
kmax=floor(Tmax/dt);
for round_cntr = 1:runs 
    if toc(starttime) > max_time, break, end
    % init round
    disp(['round ',num2str(round_cntr)])
    if runs == 1
        rng(random_seed);
    else
        rng(round_cntr);
    end
    [x0P,x0E]=starting_position(min_dist, max_dist, start_distribution, P, E);
    for i=1:NP, P(i).new_round(x0P(:,i)); end
    E.new_round(x0E);
    round_result = nan;
    XP=repmat(x0P,1,1,kmax+1);
    XE=repmat(x0E,1,1,kmax+1);
    D=zeros(1,NP);
    
    % run round
    for k=1:kmax
        for i=1:NP, P(i).plan(XP(:,:,k),XE(:,:,k)); end
        E.plan(XP(:,:,k),XE(:,:,k));
        for i=1:NP, XP(:,i,k+1) = P(i).act; end
        XE(:,1,k+1) = E.act;
        assert(isreal(XP(:,:,k+1)))
        assert(isreal(XE(:,:,k+1)))
        for i=1:NP
            D(i)=norm(XE(1:2,1,k+1)-XP(1:2,i,k+1));
            if D(i)<dc && ...
                    (E.f.name~="dubins" ||  ...
                    E.x(4)*dt*[cos(E.x(3)),sin(E.x(3))]*(P(i).x(1:2)-E.x(1:2))>0)
                % (frontal only) collision
                round_result=-k*dt; % negative value
            end
        end
        if norm(XE(1:2,1,k+1)-target)<=dc % evader reaching target
            round_result=k*dt; % positive value
        end
        if ~isnan(round_result)
            % stop round, cut containers to current length
            XP(:,:,k+2:end)=[];
            XE(:,:,k+2:end)=[];
            break;
        end
    end
    
    % store result
    results(round_cntr) = round_result;
    D=squeeze(permute(vecnorm(XP(1:2,:,:)-XE(1:2,1,:)),[3,2,1])); % cycle_count x NP matrix of distances
    V=squeeze(XE(4,1,:)); % cycle_count x 1 vector of velocities
    velocity_distance_data{round_cntr}=[V,D];
    
    % display result
    if round_result>0
        fprintf("Time to escape:\t%.6f\n",round_result)
    else
        fprintf("Time to capture:\t%.6f\n",-round_result)
    end
    fprintf("Minimal distance from the pursuers:\t%.6f\n",...
        min(min(squeeze(vecnorm(XP(1:2,:,:)-XE(1:2,:,:))))))
    if runs==1
        time_plot(dt, XP, XE);
        traj_plot(XP, XE, target);
    end
end
fprintf("Average time (successful runs):\t%.6f\n",mean(results(results>0)))
fprintf("Unsuccessful runs:\t%.6f\n",(length(results(results<0))+ sum(isnan(results)) ) )

%% save
if runs>1 && length(save_filename)>1
    save(save_filename,'results','velocity_distance_data');
end

%% nested functions
function [x0P,x0E]=starting_position(min_dist, max_dist, start_distribution, P, E)
% pursuers
NP=length(P);
d0P=rand(1,NP)*(max_dist - min_dist) + min_dist;
if start_distribution == 0
    th=linspace(-pi,pi,NP);
    x0P=[cos(th).*d0P;sin(th).*d0P];
else
    x0P=[d0P; (rand(1,NP) - 0.5)*(max_dist - min_dist)];
end
if strcmp(P(1).f.name,'dubins')
%     x0P=[x0P;rand(1,NP)*2*pi;zeros(1,NP)]; % random heading
    x0P=[x0P;zeros(1,NP);zeros(1,NP)]; % zero heading
    for i=1:NP
        x0P(end,i)=P(i).f.v_max;
    end
end

%evader
if strcmp(E(1).f.name,'dubins') % facing?
    x0E=[0;0;0;0];
else
    x0E=[0;0];
end
end

function time_plot(dt, XP, XE)
% interactive plot of the state in any time

k_max = size(XP, 3);
f = figure(1);  clf
set(f,'position',[400,150,600,600]),clf
ax = axes('Parent',f,'position',[.1 .2  .8 .7]);
k_plot=1;
dotP = plot(XP(1,:,k_plot),XP(2,:,k_plot),'*','Color','r');  hold on
dotE = plot(XE(1,:,k_plot),XE(2,:,k_plot),'*','Color','b');
mincoord=min(min([XP(1:2,:),XE(1:2,:)]));
maxcoord=max(max([XP(1:2,:),XE(1:2,:)]));
set(ax,'XLim',[mincoord,maxcoord],'YLim',[mincoord,maxcoord]);

bgcolor = f.Color;
uicontrol('Parent',f,'Style','text','Position',[50,54,23,23],...
    'String','0','BackgroundColor',bgcolor);
uicontrol('Parent',f,'Style','text','Position',[500,54,50,23],...
    'String',[num2str(k_max * dt),' s'],'BackgroundColor',bgcolor);
uicontrol('Parent',f,'Style','text','Position',[240,15,100,23],...
    'String','time','BackgroundColor',bgcolor);

slider = uicontrol('Parent',f,'Style','slider','Position',[20,40,560,20],...
    'value',k_plot, 'min',1, 'max',k_max);
slider.Callback = @(es,ed)  plot_refresh(dotP,XP(:,:,round(es.Value)),dotE,XE(:,:,round(es.Value)));

function plot_refresh(dotP,XP,dotE,XE)
set(dotP,'XData',XP(1,:));
set(dotP,'YData',XP(2,:));
set(dotE,'XData',XE(1,:));
set(dotE,'YData',XE(2,:));
end
end

function traj_plot(XP, XE, target)
% plot of agents' trajectories

figure(2), clf, hold on
plot(target(1), target(2), 'k^')
plot(squeeze(XE(1,1,:)),squeeze(XE(2,1,:)), 'color', 'b')
for i=1:size(XP, 2)
    plot(squeeze(XP(1,i,:)), squeeze(XP(2,i,:)), 'color', 'r')
end
plot(squeeze(XE(1,1,end)),squeeze(XE(2,1,end)),'.','color','k');
for i=1:size(XP, 2)
    plot(squeeze(XP(1,i,end)),squeeze(XP(2,i,end)),'.','color','k')
end
hold off
xlabel('x [m]', 'FontSize', 8)
ylabel('y [m]', 'FontSize', 8)
axis equal
l=legend({'target', 'evader', 'pursuers'}, 'location', 'southwest')
l.FontSize = 8;
folder = '/Figures/';
filename = [folder 'SVO_sim' '.fig'];
saveas(figure(1),[pwd  filename]);
filename = [folder 'SVO_sim' '.pdf'];
% filename = fullfile([pwd filename])
set(gcf, 'Units', 'Inches', 'Position', [0, 0, 3.5,3], 'PaperUnits', 'Inches', 'PaperSize', [3.5, 3])
exportgraphics(gcf, [pwd filename]);
end



