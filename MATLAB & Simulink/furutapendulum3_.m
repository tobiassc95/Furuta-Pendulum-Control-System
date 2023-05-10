clc;
close all;

%% Constants.
g=9.81;
J1=1.7e-5; L1=8.8e-2; b1=1e-6*0;
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
h=x(1);

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
s = tf('s'); %laplace variable.
P=zpk(ss(A,B,C,D))*1e-3 %plant (system) [rad/Nmm]. pole s=0 is not here (pole-zero cancellation).