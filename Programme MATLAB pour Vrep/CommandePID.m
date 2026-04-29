clc
close
clear

angle = 0;
anglep = 0;
vr=2.9;
som = 0;
consigne=-0.21;
consigneS= 0;
errs=0;
errsp=0;
soms=0;
vars=0;
err=0;
i=0;
Ts = 0.01;
kp =6* .1;
ki=kp*.5;
kd=ki*.015;

% kp = 0.1;
% ki=kp*.4;
% kd=ki*0.000925;

errp= 0;
vrep = remApi('remoteApi');
vrep.simxFinish(-1);
clientID = vrep.simxStart('127.0.0.1', 19997, true, true, 5000, 5);
vrep.simxGetStringSignal(clientID,'acceldata',vrep.simx_opmode_streaming);
vrep.simxGetStringSignal(clientID,'gyrodata',vrep.simx_opmode_streaming);
% Début de l'initialisation des joint et de la transmission
if clientID > -1
    disp('connected')
    vitesse=[-1 -1];
    [returncode1, vitessel] = vrep.simxGetObjectHandle(clientID, 'L1', vrep.simx_opmode_blocking);
    [returncode2, vitesser] = vrep.simxGetObjectHandle(clientID, 'L2', vrep.simx_opmode_blocking);
    [returncode5, guidon] = vrep.simxGetObjectHandle(clientID, 'L3', vrep.simx_opmode_blocking);
    
 	    
while true
   
[errorCode,aValue]=vrep.simxGetStringSignal(clientID,'acceldata',vrep.simx_opmode_buffer);
[errorCode,gValue]=vrep.simxGetStringSignal(clientID,'gyrodata',vrep.simx_opmode_buffer);
if (errorCode==vrep.simx_return_ok)

    a=vrep.simxUnpackFloats(aValue);
    g=vrep.simxUnpackFloats(gValue);
    % Conversion des lectures du gyroscope
    gx=g(1)*0.1/131;
    gy=g(2)*0.1/131;
    gz=g(3)*0.1/131;    
    gyr=[gx gy gz];
    % Conversion des lectures de l'acceleromètre
    ap=180*atan(a(1)/(a(2)^2+a(3)^2)^1/2)/pi;
    ar=180*atan(a(2)/(a(3)^2+a(1)^2)^1/2)/pi;
    ay=180*atan(a(3)/(a(2)^2+a(1)^2)^1/2)/pi;
    acc=[ap ar ay];
    %Filtre de fusion des lectures des deux capteurs
    angle=(0.98*(angle+gy)+0.02*ap);
    angle
    pause(.01);
    err=consigne - angle;
    som=+err*Ts;
    var=err-errp/Ts;    
    out=kp*(err)+ki*(som)+kd*(var);
    errp=err;
%     if (out>angle)
%         out=vr;
%     else
%         if (out<=-vr)
%             out=-vr;
%         end
%     end
% out
%  errs = consigneS - out;
%  soms=+ errs;
%  vars=errs - errsp;
%  
%  vitesse = kp*errs+ki*soms  + kd*vars;
%  vitesse
%  
      out=10*(out);
%      if out>=5 out = 7; end
%      if out<=-5 out = -7; end

     
 	    vrep.simxSetJointTargetVelocity(clientID, vitessel, out, vrep.simx_opmode_blocking);
  	    vrep.simxSetJointTargetVelocity(clientID, vitesser, -out, vrep.simx_opmode_blocking);
     


end
end
else
    disp('Failed connecting to remote API server');
    pause();
end
