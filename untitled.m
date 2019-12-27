clc;
clear;
t=linspace(0,20,500);
theta0=[0.5 0 0 0];I1=1;I2=1;m1=1;m2=1;L1=1;L2=1;Lc1=1;Lc2=1;g=9.8;
[theta1, theta2,dtheta1,dtheta2]= ode45_ex(theta0,I1,I2,m1,m2,L1,L2,Lc1,Lc2,g);
x1=-L1*cos(theta1);
y1=L1*sin(theta1);
x2=-L1*cos(theta1)-L2*cos(theta1+theta2);
y2=L1*sin(theta1)+L2*sin(theta1+theta2);
figure(1)
hold on;
subplot(2,2,1),plot(theta1,theta2);
xlabel('theta1(rad)'),ylabel('theta2(rad)'),title('phase potrait1');
subplot(2,2,2),plot(theta1,dtheta1);
xlabel('theta1(rad)'),ylabel('dtheta1(rad/s)'),title('phase potrait2');
subplot(2,2,3),plot(theta2,dtheta2);
xlabel('theta2(rad)'),ylabel('dtheta2(rad/s)'),title('phase potrait3');
subplot(2,2,4),plot(dtheta1,dtheta2);
xlabel('dtheta1(rad/s)'),ylabel('dtheta2(rad/s)'),title('phase potrait4');
hold off;
figure(2)
kineticv=0.5*(m1*Lc1^2*dtheta1.^2+m2*(L1^2*dtheta1.^2+Lc2^2*(dtheta1+dtheta2).^2+2*Lc2*L1*dtheta1.*(dtheta1+dtheta2)).*cos(theta2));
kinetict=0.5*(I1*dtheta1.^2+I1*(dtheta1+dtheta2).^2)+kineticv;
position=-g*(m1*Lc1*cos(theta1)+m2*(L1*cos(theta1)+Lc2*cos(theta1+theta2)));
energy=position+kinetict;
hold on
plot(t,position);
plot(t,kinetict);
plot(t,energy);
xlabel('t(s)');ylabel('energy(J)');
title('energy diagram');
legend('U','K','U+K');
hold off 
figure(3)
   Ncount=0;
   fram=0;
   
     for i=1:length(theta1)
         Ncount=Ncount+1;
         fram=fram+1;
         plot(0, 0,'.','markersize',20);
         hold on
         plot(y1(i),x1(i),'.','markersize',20);
         plot(y2(i),x2(i),'.','markersize',20);
         hold off
         line([0 y1(i)], [0 x1(i)],'Linewidth',2);
         axis([-(L1+L2) L1+L2 -(L1+L2) L1+L2]);
         line([y1(i) y2(i)], [x1(i) x2(i)],'linewidth',2);
         h=gca; 
         get(h,'fontSize') ;
         set(h,'fontSize',12);
         xlabel('Y','fontSize',12);
         ylabel('X','fontSize',12);
         title('dual pendulum','fontsize',14)
         fh = figure(3);
         set(fh, 'color', 'white'); 
         F=getframe;
         end
      movie(F,fram,20)


function [theta1, theta2,dtheta1,dtheta2] =ode45_ex(theta0,a0,b0,c0,d0,e0,f0,g0,h0,i0)
I1=a0;I2=b0;m1=c0;m2=d0;L1=e0;L2=f0;Lc1=g0;Lc2=h0;g=i0;
t=linspace(0,20,500);
[t,theta] = ode45(@smd,t,theta0);
theta1=theta(:,1);
theta2=theta(:,2);

hold on;
plot(t,theta1);
plot(t,theta2,'r');
dtheta1=theta(:,3);
dtheta2=theta(:,4);
hold off;

function dtheta = smd(t,theta)
dtheta(1,:) = theta(3);
dtheta(2,:) = theta(4);

a1=I1+I2+m1*Lc1^2+m2*(L1^2+Lc2^2+2*L1*Lc2*cos(theta(2)));
b1=I2+m2*(Lc2^2+L1*Lc2*cos(theta(2)));
c1=I2+m2*(Lc2^2+L1*Lc2*cos(theta(2)));
d1=I2+m2*Lc2^2;
e1=m1*g*Lc1*sin(theta(1))+m2*g*(L1*sin(theta(1))+Lc2*sin(theta(1)+theta(2)))-m2*L1*Lc2*sin(theta(2))*theta(3)*(2*theta(3)+theta(4));
f1=m2*g*Lc2*sin(theta(1)+theta(2))+m2*L1*Lc2*sin(theta(2))*theta(3)^2;

dtheta(3,:)= (-e1*d1+b1*f1)/(a1*d1-c1*b1) ;
dtheta(4,:)= (-a1*f1+c1*e1)/(a1*d1-c1*b1) ;
end
end
