clc
close
clear

consigne=-0.21;
err=0;
i=0;
u =0.01;
angle = 0;
anglep=0;
Kmpc = -1670.895;
Km = -10;
Bm= 29;
Np = 10;

% anglep=0;
% Kmpc = 0.195;
% Km = -17;
% Bm= 229;
% Np = 25;

% Kmpc = 0.0125;
% Km = -2;
% Bm= 1.5;
% Np = 15;

% Kmpc = 0.1;
% Km = -3.5;
% Bm= 3.5;
% Np = 15;


vrep = remApi('remoteApi');
vrep.simxFinish(-1);
clientID = vrep.simxStart('127.0.0.1', 19997, true, true, 5000, 5);
vrep.simxGetStringSignal(clientID,'acceldata',vrep.simx_opmode_streaming);
vrep.simxGetStringSignal(clientID,'gyrodata',vrep.simx_opmode_streaming);
vrep.simxStartSimulation(clientID,vrep.simx_opmode_oneshot);
% Début de l'initialisation des joint et de la transmission
if clientID > -1
    disp('connected')
    vitesse=[-1 -1];
    [returncode1, vitessel] = vrep.simxGetObjectHandle(clientID, 'L1', vrep.simx_opmode_blocking);
    [returncode2, vitesser] = vrep.simxGetObjectHandle(clientID, 'L2', vrep.simx_opmode_blocking);
    [returncode5, guidon] = vrep.simxGetObjectHandle(clientID, 'L3', vrep.simx_opmode_blocking);
    [returncode4, objectHandle] = vrep.simxGetObjectHandle(clientID, 'base_link_respondable', vrep.simx_opmode_blocking);
%% Affichage sur graphe    
plotTitle = 'lecture de l''angle';
xLabel ='temps';
yLabel = 'angle';
legend1 ='accel';
legend2 = 'gyro';
legend3='anglef';
legend4 ='commande';
ymax = 1;
ymin =0; 
plotGrid = 'on';
min =0;
max = 1;
delay = .01;
r=0.34;
D=2;
forceAndTorque=[1.0,2.0,3.0,4.0,5.0,6.0];
time = 0;
data = 0;
data1 = 0;
data2 = 0;
count =0;
com =0;
out =0;

%  vrep.simxCallScriptFunction(clientID,"base_link_respondable",...
%     vrep.sim_scripttype_childscript,"addForceAndTorque_function",1,objectHandle,...
%     6,forceAndTorque,vrep.simx_opmode_blocking);

plotGraph =plot(time,data,'-r');
hold on
plotGraph1 =plot(time,data1,'-b');
plotGraph2 =plot(time,data2,'-g');
plotGraph3=plot(time,com,'-y');
title(plotTitle,'Fontsize',15);
xlabel(xLabel,'FontSize',15);
ylabel(yLabel,'FontSize',15);
legend(legend1,legend2,legend3,legend4);
axis([ymin ymax min max])
grid(plotGrid);
% kk=0;
kk=1.45;
tic
while true
        
[errorCode,aValue]=vrep.simxGetStringSignal(clientID,'acceldata',vrep.simx_opmode_buffer);
[errorCode,gValue]=vrep.simxGetStringSignal(clientID,'gyrodata',vrep.simx_opmode_buffer);
if (errorCode==vrep.simx_return_ok)
    
    a=vrep.simxUnpackFloats(aValue);
    g=vrep.simxUnpackFloats(gValue);
    gx=g(1)*0.1/131;
    gy=g(2)*0.1/131;
    gz=g(3)*0.1/131;    
    gyr=[gx gy gz];
    ap=180*atan(a(1)/(a(2)^2+a(3)^2)^1/2)/pi;
    ar=180*atan(a(2)/(a(3)^2+a(1)^2)^1/2)/pi;
    ay=180*atan(a(3)/(a(2)^2+a(1)^2)^1/2)/pi;
    acc=[ap ar ay];
    angle=(0.98*(angle+gy)+0.05*ap);
    angle

    count =count +1;
    time(count)=toc;
    data (count) =ap(1);
    data1(count)=g(2);
    data2(count)=angle(1);
    com(count)=out(1);
    
   
    set(plotGraph3,'XData',time,'YData',com)
    set(plotGraph,'XData',time,'YData',data)
    set(plotGraph1,'XData',time,'YData',data1)
    set(plotGraph2,'XData',time,'YData',data2)
    axis([0 time(count) min max])
    
   
%     pause(.0001);
 
%     if (angle>=3.40)
%         angle=3.40;
%     else
%         if (angle<=-1.75)
%             angle=-1.75;
%         end
%     end
err=consigne - angle;
angle = angle + kk*err; %(anglep-angle);
anglep = angle;
out =zeros(1,Np);
for i = 1:Np
    err(i+1) = Km *err(i) + Bm*u;
out =1* Kmpc*err;
end

if out(1) <= -5
    out(1)=-5;
end
if out(1) >= 5
    out(1)=5;
end
% if (out(1) < 5) || (out(1) > -5)
% out(1)=out(1);
% end
    
    
%   
% ts =timeseries(angle,1:.01:999);
%      
	    vrep.simxSetJointTargetVelocity(clientID, vitessel, -out(1), vrep.simx_opmode_streaming);
  	    vrep.simxSetJointTargetVelocity(clientID, vitesser, out(1), vrep.simx_opmode_streaming);
%         plot(ts,'b')
        i=i+1;

 pause(0.01)
end
end
else
    disp('Failed connecting to remote API server');
    pause();
end
