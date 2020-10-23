syms tz tp Kc positive
% want wc = 15 rad/s
eqn1 = 1/sqrt(tz*tp) == 15;
eqn2 = sin(deg2rad(80)) == (tz-tp)/(tz+tp);
[tz, tp] = solve(eqn1, eqn2, tz, tp);
% 20*log(K*tz/tp)/2 = 73.5
Kc = (10^7.35)*tp/tz;

% 20*log(K) = 50
Kc = 10^(50/20)