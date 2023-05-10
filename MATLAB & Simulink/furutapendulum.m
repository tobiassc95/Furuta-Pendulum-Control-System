clear; %clear the workspace.
clc;
close all;

%% Constants.
g=9.81;
J1=1.75e-5; L1=8.787e-2; b1=0;
J2=4.29e-5; m2=3.8261e-3; l2=8.67e-2; b2=1e-5;
J0=J1+m2*L1^2;

%% Balance point.
xo=[0,0,pi,0];
uo=0;
yo=xo(3);

%% State variables.
A=[0,1,0,0;
   0,-b1*J2/(J0*J2-(m2*l2*L1)^2),g*L1*(m2*l2)^2/(J0*J2-(m2*l2*L1)^2),-b2*m2*l2*L1/(J0*J2-(m2*l2*L1)^2);
   0,0,0,1;
   0,-b1*m2*l2*L1/(J0*J2-(m2*l2*L1)^2),g*m2*l2*J0/(J0*J2-(m2*l2*L1)^2),-b2*J0/(J0*J2-(m2*l2*L1)^2)];

B=[0;
   J2/(J0*J2-(m2*l2*L1)^2);
   0;
   m2*l2*L1/(J0*J2-(m2*l2*L1)^2)];

C=[0,0,1,0];

D=0;

% B=[0,0;
%    J2/(J0*J2-(m2*l2*L1)^2),m2*l2*L1/(J0*J2-(m2*l2*L1)^2);
%    0,0;
%    m2*l2*L1/(J0*J2-(m2*l2*L1)^2),J0/(J0*J2-(m2*l2*L1)^222150)];\
%
% C=[1,0,0,0;
%    0,0,1,0];
% 
% D=[0;0];

%% Internal stability analize.
eig(A) %[0;0;77.1535;0]->unstable system.

%% Transfer function.
P=zpk(ss(A,B,C,D))*1e-3 %plant (system) [rad/Nmm]. pole s=0 is not here (pole-zero cancellation).

% %% Controllability & observability.
% Cm=ctrb(A,B); %controllability matrix.  
% rank(Cm) %3->uncontrollable system (3 of 4 eigenvalues (poles) can be arbitrarily placed).
% Om=obsv(A,C); %observability matrix.
% rank(Om) %3->unobservable system (3 of 4 eigenvalues (poles) can be arbitrarily placed). it was obvious since rank(Cm)=3.
% 
% %% Plant factorization.
% Ts=0.005; %sample period.
% s = tf('s'); %laplace variable.
% %cell2mat(P.p)
% Pap = zpk((s+P.p{1}(2))/(s-P.p{1}(2))); %P.p{1} = cell2mat(P.p)
% Pmp = zpk(P.k*(s-P.z{1}(1))/((s-P.p{1}(1))^2*(s+P.p{1}(2)))); %P.z{1} = cell2mat(P.z)
% pade = zpk((1-s*Ts/4)/(1+s*Ts/4)); %padé approximation.
% bode(P);
% title('bode(P)');
% grid on;
% 
% %% Loop shaping (Cascade control - Secundary control).
% k = 4000; %; %7.6e-3; %0.03;
% %Cc_ = zpk(k*s*(s+40)/((s+600)^2));
% %Cc_ = zpk(k*s*(s+50)/((s+800)^2)); %22
% %Cc_ = zpk(k*s*(s-P.z{1}(1))/((s+800)^2)); %20
% Cc_ = zpk(k*s^2/((s-P.z{1}(1))*(s+800))); %cancel Pmp + PI.
% %Cc_ = zpk(k*s^2/(s-P.z{1}(1))); %cancel Pmp + PI.
% %Cc_ = zpk(k*s); %cancel Pmp + PI.
% L = minreal(Cc_*pade*P)
% figure;
% bode(L);
% title('bode(L)');
% grid on;
% figure;
% %nyquist(L);
% % nyqlog(L);
% % title('nyqlog(L)');
% % figure;
% rlocus(L);
% title('rlocus(L)');
% 
% S = feedback(1,L);
% %pole(S)
% figure;
% bode(S);
% title('bode(S)');
% grid on;
% 
% T = minreal(1-S);
% %pole(T)
% figure;
% bode(T);
% title('bode(T)');
% grid on;
% figure;
% step(T);
% title('step(T)');
% grid on;
% 
% PS = minreal(P*S);
% %pole(PS)
% figure;
% bode(PS);
% title('bode(PS)');
% grid on;
% 
% CS = minreal(Cc_*S);
% %pole(CS)
% figure;
% bode(CS);
% title('bode(CS)');
% grid on;
% 
% % %after designing, get Cd.
% % %Cd = c2d(C,Ts,'tustin')
% % %Cd = c2d(C,Ts,c2dOptions('Method','tustin','PrewarpFrequency',wc/(tan(wc*Ts/2))));
% % wc = 303; %maximum sensitivity frequency (measured using bode(S)).
% % Cd_ = c2d(Cc_,Ts,c2dOptions('Method','tustin','PrewarpFrequency',wc));
% % [ACd_, BCd_, CCd_, DCd_] = ssdata(Cd_);
% 
% %% RotaryPendulum_PARAM.m
% %RotaryPendulum_PARAM    Parameter file to initialize the rotary pendulum
% % -------------------------------------------------------------------------
% % inputs    : -
% % outputs   : -
% % -------------------------------------------------------------------------
% % Copyright 2015 The MathWorks, Inc.
% % -------------------------------------------------------------------------
% 
% %% ------------------------------------------------------------------------
% %  model environment properties
% %  ------------------------------------------------------------------------
% g = 9.81;
% 
% %% ------------------------------------------------------------------------
% %  Base (just for PhysMod geometry)
% %  ------------------------------------------------------------------------
% radius_base= 40e-3;
% length_base= 10e-3;
% 
% %% ------------------------------------------------------------------------
% %  Pin (just for PhysMod geometry)
% %  ------------------------------------------------------------------------
% radius_vert_cylinder = 10e-3;
% length_vert_cylinder = 200e-3;
% 
% %% ------------------------------------------------------------------------
% %  RotaryArm
% %  ------------------------------------------------------------------------
% M = 20.3e-3;
% R = 109e-3;
% 
% me    = 0.012;              % mass encoder
% ma    = M - me;
% 
% % PhysMod geometry
% horizontal_arm_geo = [109e-3 radius_vert_cylinder*2 radius_vert_cylinder*2];
% 
% % Tradiotioanl approach parameters
% re    = 0.02;               % encoder offset
% Ino_e = ma*R^2/3;           % corrected I without encoder
% Itot  = Ino_e + me*re^2;    % corrected I with encoder
% I     = Itot;
% 
% %% ------------------------------------------------------------------------
% %  Pendulum
% %  ------------------------------------------------------------------------
% m = 3.3e-3;
% l = 183.2e-3;
% 
% % PhysMod geometry
% pendulum_geo = [5e-3 20e-3 l];
% 
% %% ------------------------------------------------------------------------
% %  Setup Pendulum_CAD_Control initial values
% %  ------------------------------------------------------------------------
% friction_on_off = 1;
% controller_on_off = 0;
% 
% gain_motor = 5.67;
