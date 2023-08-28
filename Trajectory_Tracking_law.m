function [v,w] = trajectory_tracking_controller(x,y,theta,xr,yr,xdr,ydr,xddr,yddr,v,Ts,kp1,kp2,kd1,kd2)

    %This function implements the feedback linearization based tracking
    %controller proposed in: "De Luca, Oriolo, Vendittelli - Control of Wheeled Mobile Robots: An
    %Experimental Overview" Section 5.4 - formula 5.18
    
    %xr,yr,xdr,ydr,xddr,yddr, are the positions, velocities, and
    %accelerations of the reference trajectory along the x and y axis
    
    %x,y,theta is the state of the differential-drive/unicycle robot
     
    %Ts is the sampling time
    
    %kp1,kp2,kd1,kd2 are designing parameters
    
    %the outputs v and w are the control linear and angular velocities of
    %unicyle robot

    xd=v*cos(theta);
    yd=v*sin(theta);
    
    u1=xddr+kp1*(xr-x)+kd1*(xdr-xd);
    u2=yddr+kp2*(yr-y)+kd2*(ydr-yd);
    
    a=u1*cos(theta)+u2*sin(theta);
    
    
    v=v+a*Ts;
    
    
    w=(u2*cos(theta)-u1*sin(theta))/v;
end

