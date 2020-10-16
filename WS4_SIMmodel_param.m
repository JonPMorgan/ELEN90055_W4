% Mobile pendulum simulink model parameters - ELEN90055 Workshops 4

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

%Try a Lead Compensator

Kc = 1000;                  % controller gain
zc = -6;                    % controller zero
pc = -100;                  % controller pole
C = tf(Kc*[1 -zc],[1 -pc]); % controller TF

pendCnum = Kc*[1 -zc];  % numerator of pendulum motor controller
pendCden = [1 -pc];  % denominator of pendulum motor controller

