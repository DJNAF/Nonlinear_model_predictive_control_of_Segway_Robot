% programme matlab MIMO avec animation
clc
clear 
close
%% Definition des matrices d'etat
% parametres:
%% Definition des matrices d'etat
% parametres:
%% definition des parametres de control
m_c=80;  % masse du conducteur
r=0.33;  % rayon de la roue
L=1.80;  % taille du conducteur
ks=1440; % constante de raideur
cs=350; % coefficient de viscosite
g=-9.81;  % pesanteur
mb=15;   % masse de la base
D=0.5;  % longueur de la base
m_r=5.86; % masse de la roue
J_r=m_c*r*r/2; % moment d'inertie de la roue 
J_m=1/2*0.5*0.03^2; % moment d'inertie moteur
psi=0.01; % viscosite du moteur
J_0=1/3*m_c*(2*L)^2; % moment inertie inclinaison
Jd=1/2*m_c*0.2^2; % moment inertie pilotage
r_reduc= 0.6; % rapport de reduction
rho = 1/r_reduc;
final_time=30;
% x0=[0;5*pi/180;0;0];

% matrices d'etat du systeme:
%% Matrices du systeme
 % Elements des matrices
 
m11 = m_c*r^2/4+r^2*Jd/(D^2)+m_r*r^2+J_r+J_m/rho^2;
m12 = m_c*r^2/4-r^2*Jd/(D^2)+m_r*r^2+J_r;
m13 = m_c*L*r/2-J_m/rho^2;
m33 = m_c*L^2+2*J_m/rho^2+J_0;
c11 = psi/rho^2;
k33 = -m_c*g*L;

% les matrices resultantes de la mise en equation du systeme
% sous la forme Mq''+Cq'+Kq = u sont donc donnees

M = [m11 m12 m13;...
m12 m11 m13;...
m13 m13 m33];
C = [c11 0 -c11;...
0 c11 -c11;...
-c11 -c11 2*c11];
K = [0 0 0;...
0 0 0;...
0 0 k33];

% la mise en modele d'etat de ces matrices nous donne:
% xs=Ax+Bu=(q,q')

A = [ zeros(3) eye(3);...
-inv(M)*K -inv(M)*C];
U =[1/rho*eye(2);-1/rho -1/rho];
B = [ zeros(3);inv(M)]*U;
% B_d = B*D;
% changement des coordonnees,pour decouplage.
% xb = r*(alpha + beta)/2
% delta = r*(alpha-beta)/D
% xb r/2 r/2 0 alpha
% delta = r/D -r/D 0 beta
% theta_P 0 0 1 theta_P
%syms r D
%Matrice de passage : S

S = [ r/2 r/2 0;...
r/D -r/D 0;...
0 0 1];

% changement des coordonnees ,pour trouver l'angle du moeur
% alpha = theta_c+rho*alpha_mot
% beta = theta_c+rho*beta_mot
% theta_c = theta_c

Smot=[ rho 0 1;...
0 rho 1;...
0 0 1];

% le systeme avec les nouvelles coordonnees  [xb delta theta_c]

M_s = M*inv(S);
C_s = C*inv(S);
K_s = K*inv(S);
A_s = [ zeros(3) eye(3);...
-inv(M_s)*K_s -inv(M_s)*C_s];
B_s = [ zeros(3);inv(M_s)]*U;

%decouplage de l'entree du systeme
% tau_theta = 1 1 tau_L
% tau_delta 1 -1 tau_R

Dec = inv([1 1;1 -1]);

%matrice decouplee B_s 

B_sd = B_s*Dec;

%Rearrangement du systeme sous une forme plus convenable
% le nouveau vecteur d'etat du systeme : [xb delta theta_c dot_xb dot_delta dot_theta_c]’
% repositionnement du vecteur d'etat:[xb theta_c dot_xb dot_theta_c delta dot_delta]’
N=[ 1 0 0 0 0 0;...
0 0 1 0 0 0;...
0 0 0 1 0 0;...
0 0 0 0 0 1;...
0 1 0 0 0 0;...
0 0 0 0 1 0];
A_sN=N*A_s*inv(N);
B_sdN=N*B_sd;

%% Systeme de reglage de la stabilite du system

 %Matrice
 deltaT=0.4;
 Ac=A_sN(1:4,1:4)+eye(4);
Bc=B_sdN(1:4,1:1);

Ac=Ac*deltaT;
Bc=Bc*deltaT;


% Discretisation des matrices d'etat
% deltaT=0.2; % Pas d'echantillonnage
% Ac=Ac*deltaT+eye(4);
% Bc=Bc*deltaT;
NT=50;N=15;n=4;m=1; 
Q=0.2e7*eye(n); QN=Q; R=0.1*eye(m);
Fx=[1 0 0 0;0 1 0 0;0 0 1 0;0 0 0 1;-1 0 0 0;0 -1 0 0;0 0 -1 0;0 0 0 -1];gx=[10;10;10;10;10;10;10;10];
 Fu=[1;-1];gu=[40;40]; 


x0=[-3;-3;pi;-1]; 
x=zeros(n,NT+1); x(:,1)=x0;
Xk=zeros(n*(N+1),1); Xk(1:n,1)=x0;
u=zeros(m,NT);
Uk=zeros(m*N,1);
zk=[Xk;Uk];

% construction de  AX,BU,QX,RU,FX,gX,FU,gU,H
for i=1:N+1
    AX((i-1)*n+1:i*n,:)=Ac^(i-1);
end
for i=1:N+1
  for j=1:N
      if i>j
          BU((i-1)*n+1:i*n,(j-1)*m+1:j*m)=Ac^(i-j-1)*Bc;
      else
          BU((i-1)*n+1:i*n,(j-1)*m+1:j*m)=zeros(n,m);
      end    
  end
end
QX=Q;RU=R;
FX=Fx;gX=gx;FU=Fu;gU=gu;
for i=1:N-1
  QX=blkdiag(QX,Q); RU=blkdiag(RU,R);
  FX=blkdiag(FX,Fx);gX=[gX;gx];
  FU=blkdiag(FU,Fu);gU=[gU;gu];
end
QX=blkdiag(QX,QN);
FX=blkdiag(FX,Fx);
gX=[gX;gx];
H=blkdiag(QX,RU);

% simulation commande  MPC
for k=1:NT
   xk=x(:,k);  
   fun = @(z)z'*H*z;
   F=blkdiag(FX,FU);g=[gX;gU];Feq=[eye((N+1)*n) -BU];geq=AX*xk;
   lb=[];
   ub=[];
   z=fmincon(fun,zk,F,g,Feq,geq,lb,ub);
   u(:,k)=z((N+1)*n+1:(N+1)*n+m,1);
   x(:,k+1)=Ac*x(:,k)+Bc*u(:,k);
   zk=z;
end    

% visualisation
figure(1)
time = (0:NT);
subplot(2,1,1)
plot(time,x(1,:),'r.-','LineWidth',.7) 
hold on
plot(time,x(2,:),'k.-','LineWidth',.7)
hold on
plot(time,x(3,:),'g.-','LineWidth',.7)
hold on
plot(time,x(4,:),'m.-','LineWidth',.7)
legend('$x_1$','$x_2$','$x_3$','$x_4$','Interpreter','latex');
xlabel('$k$','Interpreter','latex');ylabel('$\textbf{x}_{k}$','Interpreter','latex');
grid on
ax = gca;
set(gca,'xtick',[0:5:50])
set(gca,'ytick',[-10:5:10])
ax.GridAlpha = 1
ax.GridLineStyle = ':'
subplot(2,1,2)
stairs(time(1:end-1),u(1,:),'r.-','LineWidth',.7)
legend('$u_1$','Interpreter','latex');
xlabel('$k$','Interpreter','latex');ylabel('${u}_{k}$','Interpreter','latex');
grid on
ax = gca;
set(gca,'xtick',[0:5:50])
set(gca,'ytick',[-1:.5:1])
ax.GridAlpha = 1
ax.GridLineStyle = ':'


X0=0;
Y0=0;

x=x';
t=0:.1:5;
L=3.5;
for i=1:length(t)
    Xc=x(i,1);
    Xp=x(i,1)+L*sin(pi - x(i,3));
    Yp= L*cos(pi - x(i,3));
    
    
    figure(2)
    plot([-3 5],[0 0], 'linewidth',2,'color','k');
    axis([-4 6 -10 10]);
    line([Xc Xp],[Y0 Yp],'linewidth',2,'color','b');
    hold on
    plot(Xc,Y0,'s','markersize',30,'markerfacecolor','m');
    plot(Xp,Yp,'o','markersize',1,'markerfacecolor','r');
    hold off
end


