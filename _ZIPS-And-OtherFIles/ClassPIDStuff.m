%CHortetten Nicholas
%3/24/2023

%https://ctms.engin.umich.edu/CTMS/index.php?example=Introduction&section=ControlPID#9

m = 1 ;
b = 10 ;
k = 20;
F = 1 ;

Kp = 350;
Kd = 50; 
Ki = 300; 


s = tf('s');
P1 = 1/(s^2 + b*s + k);
%step(P)

%------------------------------------

C = pid(Kp)
T = feedback(C*P,1)

t = 0:0.01:2;
%step(T,t)

%------------------------------------



%Kp = 300;
%Kd = 24;
C = pid(Kp,0,Kd)
T = feedback(C*P,1)

P2 = Kp/(s^2 + b*s + (k+Kp));

%------------------------------------

C = pid(Kp); 
T = feedback(C*P, 1) 
P3 = (Kp + Kd*s)/(m*s^2 + (b+Kd)*s^2 + (k+Kp*s + Ki))

%-----------------------------------------------------
t = 0:0.01:2;
%step(T,t)

step(P3)





