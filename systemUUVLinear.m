function dDetxdt = systemUUVLinear(tt,Detx,A,B,KpGains,KdGains)

global udott wdott

x_err = 0-Detx;
x_errdot = [0-Detx(3,1); 0-Detx(4,1); 0-udott; 0-wdott];

dDetxdtt = A*Detx + B*(KpGains*x_err + KdGains*x_errdot);
udott = dDetxdtt(3,1);
wdott = dDetxdtt(4,1);

dDetxdt = dDetxdtt;
