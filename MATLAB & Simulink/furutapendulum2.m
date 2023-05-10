clear; %clear the workspace.
clc;
close all;

%% Constants.
g=9.81;
J1=1.75e-5; L1=8.787e-2; b1=1e-6*0;
J2=4.29e-5; m2=3.8261e-3; l2=8.67e-2; b2=1e-5*0;
J0=J1+m2*L1^2;

%state variables.
x=sym('x',[4 1],'real'); %states.
u=sym('u','real'); %control action.

%differential equations.
f1=x(2);
f2=([-J2*b1,m2*L1*l2*cos(x(3))*b2,-J2^2*sin(2*x(3)),-J2*m2*L1*l2*cos(x(3))*sin(2*x(3))/2,J2*m2*L1*l2*sin(x(3))]*[x(2);x(4);x(2)*x(4);x(2)^2;x(4)^2]+[J2,-m2*L1*l2*cos(x(3)),m2^2*l2^2*L1*sin(2*x(3))/2]*[u;0;g])/(J0*J2+(J2*sin(x(3)))^2-(m2*L1*l2*cos(x(3)))^2);
f3=x(4);
f4=([m2*L1*l2*cos(x(3))*b1,-b2*(J0+J2*sin(x(3))^2),m2*L1*l2*J2*cos(x(3))*sin(2*x(3)),-sin(2*x(3))*(J0*J2+(J2*sin(x(3)))^2)/2,-(m2*L1*l2)^2*sin(2*x(3))/2]*[x(2);x(4);x(2)*x(4);x(2)^2;x(4)^2]+[-m2*L1*l2*cos(x(3)),J0+J2*sin(x(3))^2,-m2*l2*sin(x(3))*(J0+J2*sin(x(3))^2)]*[u;0;g])/(J0*J2+(J2*sin(x(3)))^2-(m2*L1*l2*cos(x(3)))^2);
f=[f1;f2;f3;f4];
h=x(3);

%balance point.
xo1=0;
xo2=0;
xo3=pi;
xo4=0;
xo=[xo1;xo2;xo3;xo4];
uo=solve(subs(f4,x,xo)==0,u);

%jacobian matrices.
%nonlinear system.
A=jacobian(f,x); %states matrix.
B=jacobian(f,u); %input matrix.
C=jacobian(h,x); %output matrix.
D=jacobian(h,u); %direct transmission matrix.
% whos;
%linear system.
A=subs(A,[x(1),x(2),x(3),x(4),u],[xo(1),xo(2),xo(3),xo(4),uo]); A=double(A); %states matrix.
B=subs(B,[x(1),x(2),x(3),x(4),u],[xo(1),xo(2),xo(3),xo(4),uo]); B=double(B); %input matrix.
C=subs(C,[x(1),x(2),x(3),x(4),u],[xo(1),xo(2),xo(3),xo(4),uo]); C=double(C); %output matrix.
D=subs(D,[x(1),x(2),x(3),x(4),u],[xo(1),xo(2),xo(3),xo(4),uo]); D=double(D); %direct transmission matrix.
% whos;

%% Internal stability analize.
eig(A) %[0;0;11.4461;-11.4461]->unstable system.

%% Transfer function.
P=zpk(ss(A,B,C,D))*1e-3 %plant (system) [rad/Nmm]. pole s=0 is not here (pole-zero cancellation).

%% Controllability & observability.
Cm=ctrb(A,B); %controllability matrix.  
rank(Cm) %4->controllable system (all eigenvalues (poles) can be arbitrarily placed (poleplacement)).
Om=obsv(A,C); %observability matrix.
rank(Om) %2->unobservable system (2 of 4 eigenvalues (poles) can be arbitrarily placed (observer)).

%% Plant factorization.
Ts=0.01; %sample period.
s = tf('s'); %laplace variable.
%cell2mat(P.p)
Pmp = zpk(P.k/(s-P.p{1}(2))^2); %P.z{1} = cell2mat(P.z)
Pap = zpk((s+P.p{1}(1))/(s-P.p{1}(1))); %P.p{1} = cell2mat(P.p)
pade = zpk((1-s*Ts/4)/(1+s*Ts/4)); %padé approximation.
bode(P);
title('bode(P)');
grid on;

%% Loop shaping (Cascade control - Secundary control).
k2 = 1.3; %0.7; %65e4;
Cc2 = zpk(k2*(s-P.p{1}(2))*(s+1)/(s*(1+s*Ts/4)^2));
%Cc2 = zpk(k*(s-P.p{1}(2))^2*(s-2*P.p{1}(2))/(s*(s+4/Ts)^3));
L2 = minreal(Cc2*pade*P)
figure;
bode(L2);
title('bode(L2)');
grid on;
figure;
nyquist(L2);
title('nyquist(L2)');
%nyqlog(L2);
%title('nyqlog(L2)');
figure;
rlocus(L2);
title('rlocus(L2)');

S2 = feedback(1,L2);
%pole(S2)
figure;
bode(S2);
title('bode(S2)');
grid on;

T2 = minreal(1-S2);
%pole(T2)
figure;
bode(T2);
title('bode(T2)');
grid on;
figure;
step(T2);
title('step(T2)');
grid on;

PS2 = minreal(P*S2);
%pole(PS2)
figure;
bode(PS2);
title('bode(PS2)');
grid on;

CS2 = minreal(Cc2*S2);
%pole(CS2)
figure;
bode(CS2);
title('bode(CS2)');
grid on;

%% Loop shaping (Cascade control - Primary control).
k1 = 8.6; %2;
Cc1 = zpk(k1/s);
L1 = minreal(Cc1*T2)
figure;
bode(L1);
title('bode(L1)');
grid on;
figure;
nyquist(L1);
title('nyquist(L1)');
%nyqlog(L1);
%title('nyqlog(L1)');
figure;
rlocus(L1);
title('rlocus(L1)');

S1 = feedback(1,L1);
%pole(S1)
figure;
bode(S1);
title('bode(S1)');
grid on;

T1 = minreal(1-S1);
%pole(T1)
figure;
bode(T1);
title('bode(T1)');
grid on;
figure;
step(T1);
title('step(T1)');
grid on;

PS1 = minreal(T2*S1);
%pole(PS)
figure;
bode(PS1);
title('bode(PS1)');
grid on;

CS1 = minreal(Cc1*S1);
%pole(CS1)
figure;
bode(CS1);
title('bode(CS1)');
grid on;

% % %after designing, get Cd.
% % %Cd = c2d(C,Ts,'tustin')
% % %Cd = c2d(C,Ts,c2dOptions('Method','tustin','PrewarpFrequency',wc/(tan(wc*Ts/2))));
% % wc = 303; %maximum sensitivity frequency (measured using bode(S)).
% % Cd_ = c2d(Cc_,Ts,c2dOptions('Method','tustin','PrewarpFrequency',wc));
% % [ACd_, BCd_, CCd_, DCd_] = ssdata(Cd_);

%% RotaryPendulum_PARAM.m
%RotaryPendulum_PARAM    Parameter file to initialize the rotary pendulum
% -------------------------------------------------------------------------
% inputs    : -
% outputs   : -
% -------------------------------------------------------------------------
% Copyright 2015 The MathWorks, Inc.
% -------------------------------------------------------------------------

%% ------------------------------------------------------------------------
%  model environment properties
%  ------------------------------------------------------------------------
g = 9.81;

%% ------------------------------------------------------------------------
%  Base (just for PhysMod geometry)
%  ------------------------------------------------------------------------
radius_base= 40e-3;
length_base= 10e-3;

%% ------------------------------------------------------------------------
%  Pin (just for PhysMod geometry)
%  ------------------------------------------------------------------------
radius_vert_cylinder = 10e-3;
length_vert_cylinder = 200e-3;

%% ------------------------------------------------------------------------
%  RotaryArm
%  ------------------------------------------------------------------------
M = 20.3e-3;
R = 109e-3;

me    = 0.012;              % mass encoder
ma    = M - me;

% PhysMod geometry
horizontal_arm_geo = [109e-3 radius_vert_cylinder*2 radius_vert_cylinder*2];

% Tradiotioanl approach parameters
re    = 0.02;               % encoder offset
Ino_e = ma*R^2/3;           % corrected I without encoder
Itot  = Ino_e + me*re^2;    % corrected I with encoder
I     = Itot;

%% ------------------------------------------------------------------------
%  Pendulum
%  ------------------------------------------------------------------------
m = 3.3e-3;
l = 183.2e-3;

% PhysMod geometry
pendulum_geo = [5e-3 20e-3 l];

%% ------------------------------------------------------------------------
%  Setup Pendulum_CAD_Control initial values
%  ------------------------------------------------------------------------
friction_on_off = 1;
controller_on_off = 0;

gain_motor = 5.67;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

