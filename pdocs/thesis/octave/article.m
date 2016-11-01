%{
For octave, initially with version 4.0.3
unconstrained case
1: time 
2: location at end of step 
3: velError
4: penetration
5: posError
6: rhs
7: velocity
8: impulse
%}
uc=[
0.533 ,  -1.467 ,0,0 ,0,0,-5.33 , 0   ;
0.550 ,  -1.558 ,0,0,0,0, -5.5 , 0   ;
0.567 ,  -1.511 , 5.67 ,-0.058 ,2.8 ,  21270 , 0.01 , 34000   ;
0.583 ,  -1.502 , 0.14 ,-0.011 , 0.54, 2570  , 0.55 , 420   ;
0.600 ,  -1.496 , -0.38,-0.002 , 0.1  , -1000, 0.38 , 0   ;
0.617 ,  -1.492 ,-0.44 , 0.004 , 0     , -1600, 0.22 , 0   ;
0.717 ,  -1.497 ,0  ,-0 ,0-0.01 , 50 , -0.01 , 400   ;
0.817 ,  -1.499 ,0,0,0,0, -0.08 , 700   ;
0.917 ,  -1.500 ,0,0,0,0, -0.001 , 1000   ;
];
subplot(2,2,1)
plot(uc(:,1),uc(:,2),"color","black");
title("Location");
subplot(2,2,3)
plot(uc(:,1),uc(:,5),"color","black");
title("posError");
subplot(2,2,2)
plot(uc(:,1),uc(:,7),"color","black");
title("Velocity");
subplot(2,2,4)
plot(uc(:,1),uc(:,3),"color","black");
title("velError");
print -dtikz "-S300,200" "uc-fig"
