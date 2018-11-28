function dDetxdt = systemUUVLinear(tt,Detx,A,B,KpGains)

dDetxdt = (A-B*KpGains)*Detx;
