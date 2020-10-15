function y = qdotmobpend(u)
% Part of simulink model WS4_SIMmodel
%
% Differential vector field for the robot state dynamics.
% 
% INPUT u:
% ************************************
% * states (generalized co-orindates)*
% ************************************
% theta - angular position of drive motor (rad)
% phi - angular position of pendulum motor (rad)
% thdot - angular speed of drive motor (rad/s)
% phdot - angular speed of pendulum motor (rad/s)
% *****************
% * system inputs *
% *****************
% taup - pendulum motor torque (Nm)
% taud - drive wheel motor torque (Nm)
%
% OUTPUT y:
% Time derivative of the state (for integration in simulink model)
% Model parameters r (wheel radius) and l (pendulum length)
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Created by M.Cantoni on 2020-10-06
%
% Updated by M.Cantoni on 2020-10-08
%         Included model parameters r and l in vector of outputs to
%         enable animation of trajectories with script pend_plot.m.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% model parameters
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
mc = 1; % kg
mw = 0.01; % kg
r = 0.06; % m 
mp = 0.07; % kg
l = 0.5; % m
Ith = (mc+2*mw+mp)*r^2 + 2*mw*(r/4)^2 % kg m^2
Iph = mp*l^2; % kg m^2
bth = 0.03; %
bph = 0.5*bth; % 
g = 9.81; % gravitational acceleration m / s^2
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% unpack u into state components and system inputs
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

theta = u(1);

phi = u(2);

sph = sin(phi);
cph = cos(phi);

thdot = u(3);

phdot = u(4);

% torque input for drive and pendulum motors
taud = u(5);
taup = u(6);

%%%%%%%%%%%%%%%%%%%%%%%%%%%
% compute state derivatives
%%%%%%%%%%%%%%%%%%%%%%%%%%%

Ithph = mp*r*l*cph;

Mq = [ Ith, Ithph; ...
       Ithph, Iph];
         
Vqqd = [ bth*thdot - mp*r*l*(phdot)^2*sph; ...
         bph*phdot - g*mp*l*sph ];

thddphdd = Mq\([taud; taup] - Vqqd);     

% output state derivatives and wheel radius and lenght parameters
y = [[thdot; phdot]; thddphdd; [r; l]];
     
end