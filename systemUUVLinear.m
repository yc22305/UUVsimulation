function dDetxdt = systemUUVLinear(tt,Detx,A,B,feedbackGain)

syms t

A = double(subs(A,t,tt));
B = double(subs(B,t,tt));

dDetxdt = (A-B*feedbackGain)*Detx;