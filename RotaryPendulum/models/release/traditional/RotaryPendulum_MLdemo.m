%% RotaryPendulum_MLdemo    This demo illustrates an application of the
%                Euler-Lagrange equation. The system of differential
%                equations derived with the EulerLagrange tool is solved
%                using ode45. 
%                Note: If the Simulink 3D Animation Toolbox is available 
%                the dynamics of the system can be visualized in the VR
%                world.
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
VF  = EulerLagrange(L,X,Q,R,par,'m','s','RotaryPendulum_ODE');

%% Solve DE numerically using ode45
% Set simulation time and initial state vector
tspan = [0 10];
X0    = [0 0 10 0]*pi/180;
% Set parameter values
RotaryPendulum_PARAM;
tau = 0;    % No external input torque

% Use created .m file to solve DE
[t, X] = ode45(@RotaryPendulum_ODE,tspan,X0,[],tau,I,m,l,R,g);

% Plot state vector
plot(t,X)
title('Rotary pendulum')
xlabel('t')
ylabel('X(t)')
legend('{\alpha}','d{\alpha}/dt','{\beta}','d{\beta}/dt')

%% Start animation if VR toolbox is available
hasVR = license('test', 'virtual_reality_toolbox');
if ~hasVR
    msgbox('You do not seem to have the Simulink 3D Animation Toolbox.',...
           'Toolbox missing');
else
    % Open the VirtualReality world and display in figure
    world = vrworld('RotaryPendulum_VR.wrl');
    open(world);
    fig   = view(world, '-internal');
    
    % Define VR nodes to be able to rotate objects
    Arm      = vrnode(world, 'BFrame');
    Pendulum = vrnode(world, 'EFrame');
    
    % Loop through state vector and re-draw VR
    for ii = 1:size(X,1)
        Arm.rotation      = [0 1 0 X(ii,1)];
        Pendulum.rotation = [0 0 1 X(ii,3)];
        vrdrawnow;
        pause(0.02);
    end
end % if statement
% End of script