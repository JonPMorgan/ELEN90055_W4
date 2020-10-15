% Mobile pendulum simulink model parameters - ELEN90055 Workshops 4

SqWaveFreq = 1/18    % frequency of test signal and reference repetition (Hz)

ThRef = 45         % (rad)

ThDotRef = 90      % (rad/s)

%%%%%%%%%%%%%%%%%%%%%%%
% PARAMETERS TO ADJUST
%%%%%%%%%%%%%%%%%%%%%%%

initcond = [0; -0.05; 0; 0] % inital conditins - theta (rad); phi (rad); thdot (rad/s); phdot (rad/s)

K = 1         % constant gain drive motor controller

pendCnum = 1  % numerator of pendulum motor controller

pendCden = 1  % denominator of pendulum motor controller
