%@Author: Cristian Tiriolo - cristian.tiriolo@concordia.ca
clear; close all;
r=0.0205; %Khepera IV wheels radius 
d=0.053;  %Khepera IV wheels-axis length

M=[[r/2 r/2];[r/d -r/d]]; %Transformation Matrix
Minv=inv(M); % Inverse Transformation Matrix


%Sampling Time
Ts=0.1;

%% TRAJECTORY PARAMETERS
x0=1; y0=0; theta0=0;
q0=[x0;y0;theta0];  %Initial conditions


%scaling factor of the trajectory
eta=0.4;
alpha=10;
k=0:Ts:2*pi*alpha*2;

xr=eta*sin(k/alpha); %trajectory position along x axis
yr=eta*sin(k/(2*alpha)); %trajetcory position along y axis


%Velocity trajectory
xdr=eta*cos(k/alpha)*(1/alpha); %trajectory velocity along x axis
ydr=eta*cos(k/(2*alpha))*(1/(2*alpha));%trajectory velocity along y axis

%Acceleration trajectory
xddr=-eta*sin(k/alpha)*(1/alpha)*(1/alpha); %trajetcory acceleration along x axis
yddr=-eta*sin(k/(2*alpha))*(1/(2*alpha))*(1/(2*alpha));%trajetcory acceleration y axis

% Orientation reference
thetar=atan2(ydr,xdr); %Trajectory Orientation

%Adjusting Orientation ( a part of tetha(k) is out of phase
thetar_diff=diff(thetar);
for i=1:length(thetar_diff)
    if thetar_diff(i)<-6
        i1=i+1;
    elseif thetar_diff(i)>6
        i2=i;
    end
end
thetar(i1:i2)=thetar(i1:i2)+2*pi; %Ts=0.15



vr=sqrt(xdr.^2+ydr.^2); %Linear velocity of the trajectory
wr=(yddr.*xdr-xddr.*ydr)./(xdr.^2+ydr.^2); %Angular velocity velocity of the trajectory


%Index for scanning the trajectory
K=1;
Kf=length(xr);


%% CONTROL PARAMETERS
%State Variables
n=3;
%Input Variables
m=2;

v0=vr(1);
v=v0;

%PID parameters
kp1=1; kp2=kp1;
kd1=0.7; kd2=kd1;


%% Input Constraints

wrmax=10; %Right wheel limits
wlmax=wrmax; %Left wheel limits
 


%% Variables to store and plot the results
tt=[];
useq=[];
qseq=q0;
wrwlseq=[];



%% Plot parameters
figure
grid
hold on
plot(xr,yr,'r')

plot(q0(1,1),q0(2,1),'b-p','MarkerIndices',[1 1],'MarkerFaceColor','yellow','MarkerSize',15)

%% RESULTS COLLECTION
compute_convergence_time=1;
convergence_time=inf;

vwseq=[];
enrg_seq=[];

%% TRACKING CONTROL SIMULATION
for i=0:Ts:10000000
    
    tt=[tt i];
    
    %Immagazino la sequenza di stato
    x=qseq(1,end);
    y=qseq(2,end);
    theta=qseq(3,end);
    
  
    
    %Computing the control law 
    [v,w]=Trajectory_Tracking_law(x,y,theta,xr(K),yr(K),xdr(K),ydr(K),xddr(K),yddr(K),v,Ts,kp1,kp2,kd1,kd2);
    
    
    %Computing the SATURATION on the linear and angular velocities starting from right
    %and left wheels angular velocities limits 
    [v,w]=unicycle_saturation(wrmax,wlmax,v,w,r,d);
    
    vw=[v;w];
   
    vwseq=[vwseq vw]; 
   
    
  
    %Unicycle/Diffdrive input transformation
    wrwl=Minv*vw;
    
    
    %Storing right and left wheels angular velocities sequence
    wrwlseq=[wrwlseq wrwl];
    
    
    %Feeding the control inputs v(k) and w(k) into the differential drive
    %model, using ode45 (Runge-Kutta solver)
    v1=vw(1); w1=vw(2);
    t=0:0.00001:Ts;
    [t,q]= ode45(@(t,q,v,w)DiffDrive(t,q,v1,w1),t,qseq(:,end));
    
    
     %Updating the state sequence
    qseq=[qseq q(end,:)'];

    
    %Plotting robot trajectory 
    plot(qseq(1,end),qseq(2,end),'b--x')
    pause(0.00000001) %A little pause to obtain a live plot 
    
    if K==Kf %If the trajetcory index reaches Kf the simulation is over
        break;
    end
    
    
    K=K+1; %Updating trajectory index
    
end




%%PLOTS 



figure
%Right wheels angular velocites 
subplot(3,1,1);
hold on;
p1=plot(tt,wrwlseq(1,:));
p1.LineWidth=2;
p2=plot(tt,ones(1,length(tt))*wrmax);
p2.LineStyle='--';
p2.LineWidth=2;
p2.Color='red';
p2=plot(tt,ones(1,length(tt))*-wrmax);
p2.LineStyle='--';
p2.LineWidth=2;
p2.Color='red';
l1=legend(p2,'\omega_{r,max}');
l1.set('FontSize',10)
grid;
xlbl1=xlabel('Time[sec]','Interpreter','latex');
ylbl1=ylabel('$\omega_r(t)[RAD/sec]$','Interpreter','latex');
ylbl1.FontSize=13;
xlbl1.FontSize=13;

%Left wheels angular velocites 
subplot(3,1,2);
hold on;
p3=plot(tt,wrwlseq(2,:));
p3.LineWidth=2;
p4=plot(tt,ones(1,length(tt))*wlmax);
p4.LineStyle='--';
p4.LineWidth=2;
p4.Color='red';
p4=plot(tt,ones(1,length(tt))*-wlmax);
p4.LineStyle='--';
p4.LineWidth=2;
p4.Color='red';

l2=legend(p4,'\omega_{l,max}');
l2.FontSize=10;
grid;
xlbl2=xlabel('$Time[sec]$','Interpreter','latex');
ylbl2=ylabel('\omega_l(t)[RAD/sec]','Interpreter','latex');
ylbl2.FontSize=13;
xlbl2.FontSize=13;


%Orientation 
subplot(3,1,3)
pl=plot(tt,qseq(3,1:end-1));
pl.LineWidth=2;
grid
% axis([0 tt(length(tt)) 0 6])
xlbl2=xlabel('$Time [sec]$','Interpreter','latex');
ylbl2=ylabel('$\theta(t)[RAD]$','Interpreter','latex');



