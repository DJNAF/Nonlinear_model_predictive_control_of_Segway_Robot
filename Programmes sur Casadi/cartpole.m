%% Programme de MPC Nonlineaire Proposé par JIODA Adolphe
          %a partir du framework CASADI 3.5
          
clc
close
clear
%inclusion de la librairie casadi
import casadi.*
%% variables initialisation
% Parametres du pendule inverse g=9.8;M=0.08;mp=0.04;l=-3;g=9.8;M=0.08;mp=0.04;l=-1.8;N=11;g=-9.8;M=0.08;mp=0.04;l=2.1;
% N=9;g=-9.8;M=8;mp=4;l=2.3;N=20;g=-9.8;M=8;mp=4;l=2; Vqleurs testees

g=-9.8;M=8;mp=4;l=2;
mt=M+mp; 
p=mp/mt; 

%% parametres de la MPC

Np=15;
T=0.2;

%% Limite sur l'entree
umax=10;
umin=-umax;
%% Basee sur casadi
%Declaration des etats, parametres et commandes

x1=MX.sym('x1');
x2=MX.sym('x2');
x3=MX.sym('x3');
x4=MX.sym('x4');
x=[x1;x2;x3;x4];
nb_et=length(x);
u=MX.sym('u');
nb_commande=length(u);

%% expression de l'équation différentielle
ode=[x2;(1/mt*(1-p*cos(x(3))))*(u-mp*g*sin(x(3))) + (mp*l/mt*(1-p*cos(x(3))))*x(4)^2*sin(x(3));x4;(g/l*(1-p*cos(x(3))^2)*sin(x(3)))-(cos(x(3))/(1-p*cos(x(3))^2)*l)*((u/mt)+(mp*l/mt)*x(4)^2*sin(x(3)))];

%% expression de la fonction dynamique de l'équation

f=Function('f',{x,u},{ode});
%% Vecteur sur la plage de prediction

U = SX.sym('U',nb_commande,Np); % Commande du système
P = SX.sym('P',nb_et + nb_et);

%% Vecteurs des états initiaux et de référence
X = SX.sym('X',nb_et,(Np+1));
X(:,1) = P(1:4); % Etat initial

%% Solution de l'équation différentielle par la methode de  Euler

for k = 1:Np
    et = X(:,k);  com = U(:,k);
    valeur_f  = f(et,com);
    et_suiv  = et+ (T*valeur_f);
    X(:,k+1) = et_suiv;
end

%% Fonction d'obtention des trajectoires predites aucours du temps
ff=Function('ff',{U,P},{X});

obj = 0; % Fonction objective
g = [];  % Vecteur de contraintes

Q = zeros(4,4); Q(1,1) = 50;Q(2,2) = 10;Q(3,3) = 10; Q(4,4) = 10;% Matrices de poids 
R = 50; % Poids pour commande

%% Calcul de la fonction de cout
for k=1:Np
    et = X(:,k);  com = U(:,k);
    obj = obj+(et-P(5:8))'*Q*(et-P(5 :8)) + com'*R*com; 
end

%% calcul des contraintes
for k = 1:Np+1   % Prise en compte du bord
    g = [g ; X(1,k)];   
    g = [g ; X(2,k)]; 
    g = [g ; X(3,k)];
    g = [g ; X(4,k)];    
end

% Transformation de la matrice de commande en vecteur
OPT_variables = reshape(U,Np,1);
nlp_prob = struct('f', obj, 'x', OPT_variables, 'g', g, 'p', P);

opts = struct;
opts.ipopt.max_iter = 10;
opts.ipopt.print_level =0;%0,3
opts.print_time = 0;
opts.ipopt.acceptable_tol =1e-4;
opts.ipopt.acceptable_obj_change_tol = 1e-6;

solver = nlpsol('solver', 'ipopt', nlp_prob,opts);
args = struct;

%%  Contraintes inégales
args.lbg = -5;  
args.ubg = 5; 
% Entrées
args.lbx(1:1:Np,1) = umin; 
args.ubx(1:1:Np,1) = umax;
% Paramétrage du problème terminé

%% Début de la simulation
t0 = 0;
%  dd=[0;pi/2;3*pi/2;pi];
%  x0=sin(dd);
x0 = [0;0;2.5; 0];    % condition initiale.x0 = [1;-1;pi; 0];[2;2;pi; 3];x0 = [2;2;pi; 2];
  xs = [0 ; 0 ;pi;0]; %cible.xs = [0 ; 0 ;pi;0],[5 ; 5 ;5;0];xs = [0 ; 0 ;pi;0];
% xs=sin(dd);

xx(:,1) = x0; % xx histoire des états
t(1) = t0;

u0 = zeros(Np,1);   

sim_tim = 50; % Temps maximal

% Début MPC
mpciter = 0;
xx1 = [];
u_cl=[];
ii=1;
%La boucle principale qui est limité par l'erreur et le nombre 
%d'itération
main_loop = tic;
while(norm((x0-xs),2) > 1e-6 && mpciter < sim_tim / T)
    args.p   = [x0;xs]; % Valeur de paramètres E/C
    args.x0 = reshape(u0',Np,1); % Valeur initiale commande
  
    sol = solver('x0', args.x0, 'lbx', args.lbx, 'ubx', args.ubx,...
            'lbg', args.lbg, 'ubg', args.ubg,'p',args.p);
    u = reshape(full(sol.x)',1,Np)';
    ff_valeur = ff(u',args.p); % Calcul de la commande optimale
    xx1(:,1:4,mpciter+1)= full(ff_valeur)';
    
    u_cl= [u_cl ; u(1,:)];
    t(mpciter+1) = t0;
    [t0, x0, u0] = init(T, t0, x0, u,f); % Initialisation du prochain pas
    
    xx(:,mpciter+2) = x0;  
    mpciter
    mpciter = mpciter + 1;
    ss_error(ii) = norm((x0-xs),2);
    ii=ii+1;
end
for (i=1:250)
xref(i)=3.14;
end
main_loop_time = toc(main_loop)
lerr=length(ss_error);
tt=1:1:lerr;
figure
plot(tt,ss_error)
title 'Evolution de l''erreur du système'
xlabel('temps(s)')
ylabel('eps')
legend('erreur')
grid on
figure
subplot(221)
 stairs(t,u_cl(:,1),'k','linewidth',1.5); axis([0 t(end) -3 3])
 title 'Evolution du couple'
 ylabel('couple')
 xlabel('temps(s)')
 legend('commande(N.m)')
 grid on
 subplot(222)
 plot(t,xx(1,1:250),'linewidth',1.5)
 title 'Evolution de la position'
 ylabel('position (m)')
 xlabel 'temps(s)'
 legend('Pos')
 grid on
 subplot(223)
 plot(t,xx(2,1:250),'linewidth',1.5)
 title 'Evolution de la vitesse'
 ylabel('vitesse lineaire(m/s)')
 xlabel 'temps(s'
 legend('Vitesse Linéaire')
 grid on
 subplot(224)
 hold on
 plot(t,xx(3,1:250),t,xref,'linewidth',1.5)
 plot(t,xx(4,1:250),t,xref,'linewidth',1.5)
 title 'Evolution de l''inclinaison/vitesse'
 ylabel('inclinaison/vitesse(rad)/(rad/s)')
 xlabel 'temps(s)'
 legend('Inclinaison','vitesse')
grid on

%% animation matlab 
X0=0;
Y0=0;

x=xx';
t=0:.1:5;
L=3.5;
for i=1:length(x)
    Xc=x(i,1);
    Xp=x(i,1)+L*sin(pi - x(i,3));
    Yp= L*cos(pi - x(i,3));
    
    
    figure(4)
    plot([-3 5],[0 0], 'linewidth',2,'color','k');
    axis([-4 6 -10 10]);
    line([Xc Xp],[Y0 Yp],'linewidth',2,'color','b');
    hold on
    plot(Xc,Y0,'s','markersize',30,'markerfacecolor','m');
    plot(Xp,Yp,'o','markersize',1,'markerfacecolor','r');
    hold off
    movieVector(i) = getframe;
end

%% enregistrement de la video
video = VideoWriter('pendule_inverse');
video.FrameRate = 10;

open(video);
writeVideo(video,movieVector);
close(video);

