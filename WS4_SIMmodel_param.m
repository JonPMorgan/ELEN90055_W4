% Mobile pendulum simulink model parameters - ELEN90055 Workshops 4
close all
clear

SqWaveFreq = 1/18;    % frequency of test signal and reference repetition (Hz)

ThRef = 45;         % (rad)

ThDotRef = 90;      % (rad/s)

%%%%%%%%%%%%%%%%%%%%%%%
% PARAMETERS TO ADJUST
%%%%%%%%%%%%%%%%%%%%%%%

initcond = [0; -0.05; 0; 0]; % inital conditins - theta (rad); phi (rad); thdot (rad/s); phdot (rad/s)

K = 1;         % constant gain drive motor controller

%%%%%%%%%%%%%%%%%%%%%%
% Nominal Model Parameters
%%%%%%%%%%%%%%%%%%%%%%

b = 1.15;
k = 20;
c = 0.0524;

%%%%%%%%%%%%%%%%%%%%%%%
% Linearised Model
%%%%%%%%%%%%%%%%%%%%%%%

G0 = tf([c],[1, b, -k]);

%%%%%%%%%%%%%%%%%%%%%%%
% Controller Model
%%%%%%%%%%%%%%%%%%%%%%%

% Try a Lead Compensator

% Kc = 22000*1.3;             % controller gain
% zc = -6;                    % controller zero
% pc = -100;                  % controller pole
% C = tf(Kc*[1 -zc],[1 -pc]); % controller TF

syms tz tp Kc positive
% want wc = 15 rad/s
eqn1 = 1/sqrt(tz*tp) == 15;
eqn2 = sin(deg2rad(80)) == (tz-tp)/(tz+tp);
[tz, tp] = solve(eqn1, eqn2, tz, tp);
% 20*log(K*tz/tp)/2 = 73.5
% Kc = 316;%1; %double(10^7.35*tp/tz);
Kc = 10^(52/20);
tz = double(tz);
tp = double(tp);
C = tf(Kc*[tz 1],[tp 1]); % controller TF

pendCnum = Kc*[tz 1];  % numerator of pendulum motor controller
pendCden = [tp 1];  % denominator of pendulum motor controller

T = G0*C/(1+G0*C);          % complimentary sensitivity TF
S = 1/(1+G0*C);             % sensitivity TF
Gol = G0*C;

close all
bode(T)
hold on
margin(Gol)
bode(S)
legend('T','\Lambda','S')


% out = sim('WS4_SIMmodel_v2017b');