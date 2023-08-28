%@Author: Cristian Tiriolo - cristian.tiriolo@concordia.ca
clear; close all
r=0.0205; %Khepera IV wheels radius 
d=0.053;  %Khepera IV wheels-axis length

M=[[r/2 r/2];[r/d -r/d]]; %Transformation Matrix
Minv=inv(M); % Inverse Transformation Matrix


% % WAYPOINT PARAMETERS
x0=0; y0=0; theta0=0;
q0=[x0;y0;theta0];  %Initial conditions

xr=1; yr=0.5;



%% CONTROL PARAMETERS

Ts=0.1; %sampling time 
%State Variables
n=3;
%Input Variables
m=2;

v0=0;
v=v0;

%PID parameters
kp1=2; kp2=12;
kd1=3; kd2=7;


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
plot(xr,yr,'b-p','MarkerIndices',[1 1],'MarkerFaceColor','red','MarkerSize',15)

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
    [v,w]=Waypoint_Tracking_law(x,y,theta,xr,yr,v,Ts,kp1,kp2,kd1,kd2);
    
    
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
    
    if sqrt(([xr;yr]-[x;y])'*([xr;yr]-[x;y]))<=0.01
        break;
    end
    
   
    
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