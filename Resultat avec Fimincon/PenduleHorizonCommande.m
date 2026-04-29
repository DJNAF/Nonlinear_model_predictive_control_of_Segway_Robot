% Programme matlab avec reduction des calculs (methode 2 utilisation de l'horizon de commande)
clear all;
close all
%% Definition des matrices d'etat
% parametres:
M=8; % masse du chariot [kg]
mp=4; % masse du pendule [kg]
l= 0.5; % hauteur du pendule [m]
g=9.98; % const de gravitation [m/s^2]
% matrices d'etat du systeme:
A=[0 1 0 0;0 0 g*mp/M 0;0 0 0 1;0 0 -g*(mp+M)/(l*M) 0];
B=[0;1/M;0;1/l*M];
% Discretisation des matrices d'etat
deltaT=0.30; % Pas d'echantillonnage
A=A*deltaT+eye(4);
B=B*deltaT;
NT=50;N=5;NC=5;n=4;m=1; 
Q=eye(n); QN=Q; R=eye(m);
Fx=[1 0 0 0;0 1 0 0;0 0 1 0;0 0 0 1;-1 0 0 0;0 -1 0 0;0 0 -1 0;0 0 0 -1];gx=[10;10;10;10;10;10;10;10];
Fu=[1;-1];gu=[15;15];




x0=[10;0;5;0];
x=zeros(n,NT+1); x(:,1)=x0;
Xk=zeros(n*(N+1),1); Xk(1:n,1)=x0;
u=zeros(m,NT);
Uk=zeros(m*NC,1);
zk=Uk;

% construction des matrices  AX,BU,QX,RU
for i=1:N+1
      AX((i-1)*n+1:i*n,:)=A^(i-1);
  for j=1:N
      if i>j
          BU((i-1)*n+1:i*n,(j-1)*m+1:j*m)=A^(i-j-1)*B;
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
H=BU(:,1:NC)'*QX*BU(:,1:NC)+RU(1:NC,1:NC);


% simulation MPC
for k=1:NT
   xk=x(:,k);
   qk=2*xk'*AX'*QX*BU(:,1:NC);rk=xk'*AX'*QX*AX*xk;
   fun = @(z)z'*H*z+qk*z+rk;
   F=[FX*BU(:,1:NC);FU(:,1:NC)];g=[gX-FX*AX*xk;gU];Feq=[];geq=[];
   lb=[];ub=[];
   z=fmincon(fun,zk,F,g,Feq,geq,lb,ub);
   u(:,k)=z(1:m,1);
   x(:,k+1)=A*x(:,k)+B*u(:,k);
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

