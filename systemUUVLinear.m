function dDetxdt = systemUUVLinear(tt,Detx,A,B,KGains)

err = 0-Detx;

dDetxdt = A*Detx + B*KGains*err;
