function commandesVrep()

%% Initialize model
vrep = remApi('remoteApi');
vrep.simxFinish(-1);
clientID = vrep.simxStart('127.0.0.1', 19997, true, true, 5000, 5);
vrep.simxGetStringSignal(clientID,'acceldata',vrep.simx_opmode_streaming);
vrep.simxGetStringSignal(clientID,'gyrodata',vrep.simx_opmode_streaming);
 %vrep.simxGetStringSignal(clientID,'angledata',vrep.simx_opmode_streaming);
% vrep.simxGetStringSignal(clientID,'vitessedata',vrep.simx_opmode_streaming);
vrep.simxStartSimulation(clientID,vrep.simx_opmode_oneshot);

%% PRISE EN CHARGE DES JOINTS ET DES OBJETS SCENIQUES DE LA SIMULATION

 if clientID > -1
    disp('connected')
    [returncode1, vitessel] = vrep.simxGetObjectHandle(clientID, 'L1', vrep.simx_opmode_blocking);
    [returncode2, vitesser] = vrep.simxGetObjectHandle(clientID, 'L2', vrep.simx_opmode_blocking);
    [returncode4, objectHandle] = vrep.simxGetObjectHandle(clientID, 'base_link_respondable', vrep.simx_opmode_blocking);
     [returncode3, objectHandle1] = vrep.simxGetObjectHandle(clientID, 'ReferenceFrame', vrep.simx_opmode_blocking);
    vrep.simxGetObjectOrientation(clientID,objectHandle,-1,vrep.simx_opmode_streaming);
    %vrep.simxGetObjectPosition(clientID,objectHandle1,-1,vrep.simx_opmode_streaming);

  
dbstop if error; % debugger break on error

global RUN_MAIN_LOOP; % the main loop runs as long as this is set to true
RUN_MAIN_LOOP = true; % set to false to stop simulation
global KB; % global ASCII keyboard input map
KB = zeros(intmax('uint8'), 1);

cleanup_figures   = onCleanup(@() eval('close all'));
cleanup_functions = onCleanup(@() eval('clear functions'));
  
    %% CONFIGURATION DE LA VISUALISATION GRAPHIQUE DES SIGNAUX ISSUS DE VREP 
    
plotTitle = 'Angles et Couples';
xLabel ='temps';
yLabel = 'angles';
legend1 ='inclinaison(theta)';
legend2 = 'orientation(delta)';
legend3='entrée (couples)';
ymax = 200;
ymin =-100; 
plotGrid = 'on';
min =-200;
max = 200;
delay = .01;
time = 0;
data = 0;
data1 = 0;
count =0;
com =0;
out =0;
plotGraph =plot(time,data,'-r');
hold on
plotGraph1 =plot(time,data1,'-b');
plotGraph3=plot(time,com,'-y');
title(plotTitle,'Fontsize',15);
xlabel(xLabel,'FontSize',15);
ylabel(yLabel,'FontSize',15);
legend(legend1,legend2,legend3);
axis([ymin ymax min max])
grid(plotGrid);


% Simulation Parameter
model.dt_ctrl_sec = 1/50;
control_every_n_epoch = 10;
model.dt_sim_sec = model.dt_ctrl_sec / control_every_n_epoch;
 epoch = control_every_n_epoch;
  model = model_init(model);
  model = control_init_linear(model);
%  model = control_init_linear_ext(model);
%  model = control_init_linear_ext_cl1norm(model);
%  model = control_init_lqr(model);
%  model = control_init_PID(model);

history_time = zeros(1000, 1);
history_u = zeros(size(history_time));
history_x = zeros(length(history_time), length(model.x));

time_sec = 0;
angle = 0
jj = 0;
epoch = jj;
while true
%  % Lecture des angle d'inclinaison et d'orientation   
 [errorCode,angleValue]=vrep.simxGetStringSignal(clientID,'angleData',vrep.simx_opmode_buffer);
% [errorCode,vitesseValue]=vrep.simxGetStringSignal(clientID,'vitesseData',vrep.simx_opmode_buffer);
[errorCode,aValue]=vrep.simxGetStringSignal(clientID,'acceldata',vrep.simx_opmode_buffer);
[errorCode,gValue]=vrep.simxGetStringSignal(clientID,'gyrodata',vrep.simx_opmode_buffer);
if (errorCode==vrep.simx_return_ok)
    
    a=vrep.simxUnpackFloats(aValue);
    g=vrep.simxUnpackFloats(gValue);
    gx=g(1)*0.1/131;
    gy=g(2)*0.1/131;
    gz=g(3)*0.1/131;    
    gyr=[gx gy gz];
    ap=180*(atan(a(1)/(a(2)^2+a(3)^2)^1/2))/pi;
    ar=180*(atan(a(2)/(a(3)^2+a(1)^2)^1/2))/pi;
    ay=180*(atan(a(3)/(a(2)^2+a(1)^2)^1/2))/pi;
    acc=[ap ar ay];
    angle=(0.98*(angle+gy)+0.02*ap);
    model.x(3) = angle;
    angle
 % Visualisation des lectures
 count =count +1;
    time(count)=toc;
    data (count) =angle;
%   data1(count)=delta;
    com(count)=out(1); 
    
    set(plotGraph3,'XData',time,'YData',com)
    set(plotGraph,'XData',time,'YData',data)
    set(plotGraph1,'XData',time,'YData',data1)
    axis([0 time(count) min max])
 u = 0.00;
if epoch+1>epoch
      
        [model, u] = model.ctrl_func(model, model.dt_ctrl_sec,u);
        epoch = 0;
        
    uEff = u;
    out = 8*u;
 
    if (model.x(3) > pi/2)
        model.x(3) = pi/2;
    end

    if model.x(3) < -pi/2
        model.x(3) = -pi/2;
    end
    model.x = fstateRK4(model.dt_ctrl_sec, model.x, u, model);
   %% APPLICATION DE LA COMMANDE AU SYSTEME
if out>=5  out  = 5; end
if out <=-5 
    out = -5 ; 
end
out
	    vrep.simxSetJointTargetVelocity(clientID, vitessel, out, vrep.simx_opmode_streaming);
  	    vrep.simxSetJointTargetVelocity(clientID, vitesser, -out, vrep.simx_opmode_streaming);
pause(0.01)
end   
 
end
% end
jj =jj+1;
end
else
    disp('Failed connecting to remote API server');
    pause();
 end
end
function [model] = model_init(model)

%% Model Parameters
model.uMin = -10;
model.uMax =  10;
model.integrator = 0;
model.previous_error = 0;
model.N = 50;
model.Nc = 50;

model.x = zeros(4,1); % [x, xdot, theta, thetadot] (m, m/s, rad, rad/s)
model.consigne = -0.5737;

end

function [x_k_dot, A, B] = fdot(x_k, u, model)

%% Definition des matrices d'etat
% parametres:
%% definition des parametres de control
m_c=80;  % masse du conducteur
r=0.25;  % rayon de la roue
L=1.80;  % taille du conducteur
ks=1440; % constante de raideur
cs=350; % coefficient de viscosite
g=-9.8;  % pesanteur
mb=20;   % masse de la base
D=0.5;  % longueur de la base
m_r=5.86; % masse de la roue
J_r=m_c*r*r/2; % moment d'inertie de la roue 
J_m=1/2*0.5*0.03^2; % moment d'inertie moteur
psi=5.7e-2; % viscosite du moteur
J_0=1/3*m_c*(2*L)^2; % moment inertie inclinaison
Jd=1/2*m_c*0.2^2; % moment inertie pilotage
r_reduc= 0.7; % rapport de reduction
rho = 1/r_reduc;

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
 A=A_sN(1:4,1:4);
B=B_sdN(1:4,1:1);


x_k_dot = A*x_k + B*u;

end

function [model] = control_init_linear(model)

model.ctrl_func = @control_run_linear;

%% Linear MPC
Np = model.N;
Nc = model.Nc;
[~, A, B] = fdot(model.x, 0, model);
Ap = eye(length(model.x)) + A*model.dt_ctrl_sec;
Bp = B*model.dt_ctrl_sec;
Cp = [0 1 0 0; 0 0 1 0];
%[Ap, Bp, Cp] = c2dm(A, B, Cp, zeros(size(Cp,1),1), model.dt_ctrl_sec);
model.lin_Ap = Ap;
model.lin_Bp = Bp;
model.lin_Cp = Cp;

model.lin_Rs = zeros(Np*size(Cp,1),1); % desired setpoint for next Np epochs

[Phi, F] = linearmpcgains(Ap,Bp,Cp,Nc,Np);
model.lin_Phi = Phi;
model.lin_F = F;
rw = 0.01; % tuning parameter
model.lin_Rbar = rw*eye(Nc);

end

function [model, u] = control_run_linear(model, dt_sec, u)

%% Linear MPC
Nc = model.Nc;
% Contrainte du système <= u <= uMax (limit amplitude):
HU = [tril(ones(Nc)); -tril(ones(Nc))];
ULIM = [ones(Nc,1)*model.uMax; -model.uMin*ones(Nc,1)];

% Weight matrix
Q = eye(size(model.lin_Phi, 1));

H = (model.lin_Phi'*Q*model.lin_Phi + model.lin_Rbar);
f = -model.lin_Phi'*Q*(model.lin_Rs - model.lin_F*model.x);

upred = qphild(H, f, HU, ULIM);
upred(upred > model.uMax) = model.uMax;
upred(upred < model.uMin) = model.uMin;

u = upred(1);

end

function [model] = control_init_linear_ext(model)

model.ctrl_func = @control_run_linear_ext;

%% Ext. Linear MPC
Np = model.N;
Nc = model.Nc;
model.x_prev = model.x;
[~, A, B] = fdot(model.x, 0, model);
Ap = eye(length(model.x)) + A*model.dt_ctrl_sec;
Bp = B*model.dt_ctrl_sec;
Cp = [0 1 0 0; 0 0 1 0];
model.lin_Ap = Ap;
model.lin_Bp = Bp;
model.lin_Cp = Cp;

model.lin_Rs = zeros(Np*size(Cp,1),1); % cible 

[Phi, F] = mpcgainEx(Ap,Bp,Cp,Nc,Np);
model.lin_Phi = Phi;
model.lin_F = F;
rw = 1.5; % reglage du poids sur l'entrée du système
model.lin_Rbar = rw*eye(Nc);

end

function [model] = control_init_linear_ext_cl1norm(model)

model.ctrl_func = @control_run_linear_ext_cl1norm;
%% Etend. Linear MPC
Np = model.N;
Nc = model.Nc;
model.x_prev = model.x;
[~, A, B] = fdot(model.x, 0, model);
Ap = eye(length(model.x)) + A*model.dt_ctrl_sec;
Bp = B*model.dt_ctrl_sec;
Cp = [0 1 0 0; 0 0 1 0];
% [Ap, Bp, Cp] = c2dm(A, B, Cp, zeros(size(Cp,1),1), model.dt_ctrl_sec);
model.lin_Ap = Ap;
model.lin_Bp = Bp;
model.lin_Cp = Cp;

model.lin_Rs = zeros(Np*size(Cp,1),1); 

[Phi, F] = mpcgainEx(Ap,Bp,Cp,Nc,Np);
model.lin_Phi = Phi;
model.lin_F = F;

CL1COST = .2;
model.lin_Astar = [model.lin_Phi; CL1COST*eye(Nc)];

end

function [model] = control_init_lqr(model)

model.ctrl_func = @control_run_lqr;

[~, A, B] = fdot(model.x, 0, model);
model.K = lqr(A, B, diag([1e-1 1e2 5e4 1e1]), 0.05);

end
function [model] = control_init_PID(model)

model.ctrl_func = @control_run_PID;


 model.Kp = -75.9;
 model.Ki = -55.5;
 model.Kd = -88.019;
 dt_sec =0.01;

end


function [model, u] = control_run_linear_ext(model, dt_sec, u)

%% Linear MPC
Nc = model.Nc;
dx = model.x - model.x_prev; % dx = x(k) - x(k-1)
model.x_prev = model.x;
y = model.lin_Cp * model.x; % y(k)
x_e = [ dx; y ]; % dx = x(k) - x(k-1)
% Contrainte sur l'entée du système uMin <= u <= uMax 
HU = [tril(ones(Nc)); -tril(ones(Nc))];
ULIM = [ones(Nc,1)*model.uMax - u; -model.uMin*ones(Nc,1) + u];
% Contraintes sur l'incrément de commande |du|<duMax
% duMax = 0.5;
% HU = [HU; eye(Nc); -eye(Nc)];
% ULIM = [ULIM; ones(Nc*2, 1)*duMax];

Q = diag(repmat([2; 1], model.N, 1));
H = (model.lin_Phi'*Q*model.lin_Phi + model.lin_Rbar);
f = -model.lin_Phi'*Q*(model.lin_Rs - model.lin_F*x_e);

%A = model.lin_Phi;
%A = [A; eye(Nc)];
%l = model.lin_Rs-model.lin_F*x_e;
%l = [l; zeros(Nc, 1)];
%DU = cl1norm(A, l, [],[], HU, ULIM);

DU = qphild(H, f, HU, ULIM);

% <DEBUG>
uhorizon = DU*0;
uhorizon(1) = u + DU(1);
for i=2:length(uhorizon)
    uhorizon(i) = uhorizon(i-1) + DU(i);
end
Ypred = model.lin_F*x_e + model.lin_Phi*DU;
% </DEBUG>

u = u + DU(1); % u(k) = u(k-1) + du(k)
u(u > model.uMax) = model.uMax;
u(u < model.uMin) = model.uMin;


end

function [model, u] = control_run_linear_ext_cl1norm(model, dt_sec, u)

%% Linear MPC
Nc = model.Nc;
dx = model.x - model.x_prev; % dx = x(k) - x(k-1)
model.x_prev = model.x;
y = model.lin_Cp * model.x; % y(k)
x_e = [ dx; y ]; % dx = x(k) - x(k-1)
HU = [tril(ones(Nc)); -tril(ones(Nc))];
ULIM = [ones(Nc,1)*model.uMax - u; -model.uMin*ones(Nc,1) + u];

l = model.lin_Rs-model.lin_F*x_e;
l = [l; zeros(Nc, 1)];
DU = cl1norm(model.lin_Astar, l, [],[], HU, ULIM);

% <DEBUG>
uhorizon = DU*0;
uhorizon(1) = u + DU(1);
for i=2:length(uhorizon)
    uhorizon(i) = uhorizon(i-1) + DU(i);
end
Ypred = model.lin_F*x_e + model.lin_Phi*DU;
% </DEBUG>

u = u + DU(1); % u(k) = u(k-1) + du(k)
u(u > model.uMax) = model.uMax;
u(u < model.uMin) = model.uMin;


end

function [model, u] = control_run_lqr(model, dt_sec, u)

%% LQR
u = -model.K * model.x;

end

function [model, u] = control_run_PID(model, dt_sec,u)
%% PID
 e = model.consigne - model.x(3);

 model.integrator = model.integrator + e*dt_sec;
 derivative = (e - model.previous_error) / dt_sec;
model_previous_error =e;
 u = model.Kp*e + model.Ki*model.integrator + model.Kd*derivative;
end

function [model] = simstep(model, dt_sec, u)
    u = min(max(u,model.uMin),model.uMax);

    if (model.x(3) > pi/2)
        model.x(3) = pi/2;
    end

    if model.x(3) < -pi/2
        model.x(3) = -pi/2;
    end

    model.x = fstateRK4(dt_sec, model.x, u, model);
end

function fnext = fstateRK4(dt_sec, x_k, u, model)

k1 = fdot(x_k, u, model);
k2 = fdot(x_k + dt_sec/2*k1, u, model);
k3 = fdot(x_k + dt_sec/2*k2, u, model);
k4 = fdot(x_k + dt_sec*k3, u, model);
fnext = x_k + dt_sec/6*(k1 + 2*k2 + 2*k3 + k4);

end

function fnext = fstateEuler(dt_sec, x_k, u, model)

k1 = fdot(x_k, u, model);
fnext = x_k + k1*dt_sec;
end

function vrepInit()
vrep = remApi('remoteApi');
vrep.simxFinish(-1);
clientID = vrep.simxStart('127.0.0.1', 19997, true, true, 5000, 5);
vrep.simxGetStringSignal(clientID,'angledata',vrep.simx_opmode_streaming);
vrep.simxGetStringSignal(clientID,'vitessedata',vrep.simx_opmode_streaming);
vrep.simxStartSimulation(clientID,vrep.simx_opmode_oneshot);

%% PRISE EN CHARGE DES JOINTS ET DES OBJETS SCENIQUES DE LA SIMULATION

if clientID > -1
    disp('connected')
    [returncode1, vitessel] = vrep.simxGetObjectHandle(clientID, 'w1', vrep.simx_opmode_blocking);
    [returncode2, vitesser] = vrep.simxGetObjectHandle(clientID, 'w2', vrep.simx_opmode_blocking);
    [returncode4, objectHandle] = vrep.simxGetObjectHandle(clientID, 'base_link_respondable', vrep.simx_opmode_blocking);
     [returncode3, objectHandle1] = vrep.simxGetObjectHandle(clientID, 'ReferenceFrame', vrep.simx_opmode_blocking);
    vrep.simxGetObjectPosition(clientID,objectHandle,-1,vrep.simx_opmode_streaming);
    vrep.simxGetObjectPosition(clientID,objectHandle1,-1,vrep.simx_opmode_streaming);

    %% CONFIGURATION DE LA VISUALISATION GRAPHIQUE DES SIGNAUX ISSUS DE VREP 
    
plotTitle = 'Angles et Couples';
xLabel ='temps';
yLabel = 'angles';
legend1 ='inclinaison(theta)';
legend2 = 'orientation(delta)';
legend3='entrée (couples)';
ymax = 200;
ymin =-100; 
plotGrid = 'on';
min =-200;
max = 200;
delay = .01;
time = 0;
data = 0;
data1 = 0;
count =0;
com =0;
tic
A = 'angleData';
V='vitesseData';

plotGraph =plot(time,data,'-r');
hold on
plotGraph1 =plot(time,data1,'-b');
plotGraph3=plot(time,com,'-y');
title(plotTitle,'Fontsize',15);
xlabel(xLabel,'FontSize',15);
ylabel(yLabel,'FontSize',15);
legend(legend1,legend2,legend3);
axis([ymin ymax min max])
grid(plotGrid);



else
    disp('Failed connecting to remote API server');
    pause();
end

end

function VREP = lecture()
    angleV=vrep.simxUnpackFloats(angleValue);
    vitesse = vrep.simxUnpackFloats(vitesseValue);
    VREP.theta = angleV(2);
    VREP.delta = angleV(3);
 % Visualisation des lectures
 count =count +1;
    time(count)=toc;
    data (count) =VREP.theta;
    data1(count)=VREP.delta;
    com(count)=VREP.out(1); 
    
    set(plotGraph3,'XData',time,'YData',com)
    set(plotGraph,'XData',time,'YData',data)
    set(plotGraph1,'XData',time,'YData',data1)
    axis([0 time(count) min max])
end

function commande(out,clientID)
%% APPLICATION DE LA COMMANDE AU SYSTEME
if out>=5  out  = 5; end
if out <=-5 
    out = -5 ; 
end
	    vrep.simxSetJointTargetVelocity(clientID, vitessel, -out, vrep.simx_opmode_streaming);
  	    vrep.simxSetJointTargetVelocity(clientID, vitesser, out, vrep.simx_opmode_streaming);
end




