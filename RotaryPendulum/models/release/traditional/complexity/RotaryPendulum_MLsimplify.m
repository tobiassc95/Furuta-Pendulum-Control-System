%% RotaryPendulum_MLsimplify This demo illustrates an application of the
%                Euler-Lagrange equation.
%
% Copyright 2015 The MathWorks, Inc.

%% Derive differential equations using Euler-Lagrange
syms I m l Rl g al dal be dbe tau
T   = I*dal^2/2 + m/2*(Rl^2*dal^2 + l^2*dal^2*sin(be)^2/3 + ...
      l^2*dbe^2/4 - Rl*l*dal*dbe*cos(be)) + m*l^2*dbe^2/24;
V   = m*g*l/2*cos(be);
L   = T - V;
R   = 0;
X   = {al dal be dbe};
Q   = {tau 0};
par = {I m l Rl g};
VF  = EulerLagrange(L,X,Q,R,par);

%% Simplify to physical pendulum: I -> inf, dal = 0
VF_PhysPendulum = limit(VF,I,inf);
VF_PhysPendulum = subs(VF_PhysPendulum, dal, 0);
fprintf('Physical pendulum:\n')
disp(VF_PhysPendulum)

%% Small oscillations about stable equilibrium
syms gamma
VF_PhysPendulum = subs(VF_PhysPendulum, be, gamma-pi);
VF_PendulumAppr = taylor(VF_PhysPendulum,gamma,'Order',2,'ExpansionPoint',0);
fprintf('\n Small angles approximation:\n')
disp(VF_PendulumAppr)
% End of script