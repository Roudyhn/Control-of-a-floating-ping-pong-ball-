%% 
%write the experimentally obtained transfer function
num=1.602;
den= [1 0.2235 0];
G=tf(num,den)
step(G)
step(feedback(G,1))
stepinfo(feedback(G,1))
% writing our desired criteria , concerning the OS and Ts
Mp=0.1;
Ts=2.5
A=(log(Mp))^2
zeta= ((A)/(A+(pi)^2))^0.5;
Wn=(4)/(zeta*Ts);
s1desired= -(zeta*Wn) + (Wn*(1-zeta^2)^0.5)*j
s2desired= -(zeta*Wn) - (Wn*(1-zeta^2)^0.5)*j
%drawing our Root locus 
rlocus(G);
sgrid(zeta, Wn);
%from the intersection of zeta and wn it is clear that we are not on the
%root locus 
%Hence we need to design for our controller 
%%
%designing a pd controller 
%computing the angle of defficiency :
%our poles are ( 0 and 0.2235 )
phi = 180 - (180 - atand(2.182/1.6)) - (180 - atand(2.182/(1.6-0.2235)))

%phi=-68.4971
%Hence I should compensate by 68.4971
% the derivative controller has the form of kd(s+z)
z=1.6 + ((2.182)/(tand(68.4971)))
%z=2.459
num1=[1 2.459];
den1=1;
Gc1=tf(num1,den1);
sys1=Gc1*G;
rlocus(sys1)
sgrid(zeta,Wn)
hold on
[K, poles] = rlocfind(sys1); 
%find kd from the magnitude condition , replace s by s desired :
%by substituting we find that kd =1.85
numt1=[2 2.1]
dent=1
Gct=tf(numt1,dent)
kd=1.85
sys1=kd*sys1
step(feedback(sys1,1))
stepinfo(feedback(sys1,1))
syst=Gct*G*kd
step(feedback(syst,1))
stepinfo(feedback(syst,1))
%Ts=2.5393
%Mp=22.8821
s = tf('s');
ess1=limit(s*(1/(1+(sys1)*1/s)),s,0);
%I was not able to get a value, because I do not have the symbolic Math
%toolbox
%Its clearly visible that our root locus intersects with our desired point 
%%
%design of a PID controller .
%previously we were able to obtain the angle of defficiency phi=-68.4971
%Gc2(s) = (k(s+z1)(s+z2))/s
%let z1=0
%z2=z ; previously determined from the angle of defficiency 
z2=2.459
%Gc2(s) = (k* (s)(s+2.459)) / s
%k is determined from the magnitude condition 
%k= magnitude (s(s^2 + 0.2235 s ))/((s(s+2.459)*1.6) evaluated at s desired
k=1.86;
num2=[1 2.459 0];
den2=[1 0]
Gc2ol=tf(num2,den2)
Gc2=k*Gc2ol
sys2=Gc2*G
step(feedback(sys2,1))
stepinfo(feedback(sys2,1))
%Overshoot = 22.8712
%Ts = 2.5365 
s = tf('s');
ess2=limit(s*(1/(1+(sys2)*1/s)),s,0);
rlocus(sys2);
%as you can notice the root locus has shifted to the right 
sgrid(zeta,Wn);
%our desired points intersect with our root locus 
%%
%The overshoot and settling time are technically very close , but the pd is
%considered to be more stable because the root locus is shifted to the
%right 
