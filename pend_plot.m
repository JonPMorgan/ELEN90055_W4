%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Script for producing pendulum-platform animation from data 
% generated by Simulink model WS4_SIMmodel
%
% Uses out.phsim, out.thsim, out.mobpenrm out.mobpenl in workspace
%
% Generates movie in varible mobpendMov and plays it once. 
% To play movie again use 
% mobpendFig = figure; movie(mobpendFig,mobpendMov,1,1/dt);
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Created by M.Cantoni on 2020-10-06
%
% Updated by M.Cantoni on 2020-10-09
%         Added pre-processing of data to make sampled 
%         equally spaced across time
%
% Updated by M.Cantoni on 2020-10-10
%         Added movie for real time playback
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


dt = 0.05;
loops = round(max(out.phsim.time)/dt);

phs = [];
ths = [];

for i = 1:loops
    ind = find(out.phsim.time <= (i-1)*dt);
    phs = [phs out.phsim.data(max(ind))];
    ths = [ths out.thsim.data(max(ind))];
end

xc = 180\pi*out.mobpenr(1)*ths;
yc = zeros(loops); 
xp = xc + out.mobpenl(1)*sin(180\pi*phs);
yp = out.mobpenl(1)*cos(180\pi*phs);

xmin = min(min(xc,xp));
xmax = max(max(xc,xp));
delta = 0.01;

figure;
plot(xmin-delta,-0.6,xmax+delta,0.6);
xlabel('Position along the rail');
ylabel('Position above/below the rail');
title('Pendulum (circle) on a mobile platform (square)');
axis tight manual
ax = gca;
ax.NextPlot = 'replaceChildren';

clear mobpendMov;
mobpendMov(loops) = struct('cdata',[],'colormap',[]);

for i = 1:loops
    plot(xc(i),yc(i),'ks',xp(i),yp(i),'ko','LineWidth',2,'MarkerSize',12);
    drawnow
    mobpendMov(i) = getframe(gcf);
end

close gcf;

mobpendFig = figure; movie(mobpendFig,mobpendMov,1,1/dt);


